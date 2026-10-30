using System;
using System.Buffers;
using System.Collections.Generic;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Physics.Dynamics.Contacts;

namespace Robust.Shared.Physics.Systems;

public abstract partial class SharedPhysicsSystem
{
    // Contacts that can still change this substep: awake bodies, fresh pairs, filter dirties,
    // disabled collision, or nullspace. Sleeping same-broadphase pairs are unlinked after the
    // first pass, which is what keeps a settled pile from dominating Collide.
    private Contact? _collideHead;
    private Contact? _collideTail;
    private int _collideCount;
    private uint _transformStamp = 1;
    private readonly Dictionary<EntityUid, Matrix3x2> _broadphaseWorldMatrices = new();

    private static Contact[] GrowContacts(Contact[] contacts, int index)
    {
        if (index < contacts.Length)
            return contacts;

        var grown = ArrayPool<Contact>.Shared.Rent(Math.Max(contacts.Length * 2, index + 1));
        Array.Copy(contacts, grown, contacts.Length);
        ArrayPool<Contact>.Shared.Return(contacts, true);
        return grown;
    }

    private Transform CachedPhysicsTransform(PhysicsComponent body, EntityUid uid, TransformComponent xform)
    {
        if (body.TransformStamp != _transformStamp)
        {
            body.TransformStamp = _transformStamp;
            body.TransformCache = GetPhysicsTransform(uid, xform);
        }

        return body.TransformCache;
    }

    private Matrix3x2 BroadphaseWorldMatrix(EntityUid broadphase)
    {
        if (_broadphaseWorldMatrices.TryGetValue(broadphase, out var matrix))
            return matrix;

        matrix = _transform.GetWorldMatrix(XformQuery.GetComponent(broadphase));
        _broadphaseWorldMatrices.Add(broadphase, matrix);
        return matrix;
    }

    /// <summary>
    /// Contacts that still need overlap or manifold work. Sleeping pairs that have already been
    /// generated stay out of this list until a body wakes, collision is disabled, or a filter changes.
    /// </summary>
    private static bool ContactNeedsCollide(Contact contact)
    {
        if (!contact.Generated)
            return true;

        if ((contact.Flags & ContactFlags.Filter) != 0)
            return true;

        if ((contact.Flags & (ContactFlags.Deleting | ContactFlags.Deleted)) != 0)
            return false;

        var bodyA = contact.BodyA;
        var bodyB = contact.BodyB;
        if (bodyA == null || bodyB == null)
            return true;

        if (!bodyA.CanCollide || !bodyB.CanCollide)
            return true;

        var xformA = contact.XformA;
        var xformB = contact.XformB;
        if (xformA == null || xformB == null ||
            xformA.MapID == MapId.Nullspace || xformB.MapID == MapId.Nullspace)
            return true;

        if (bodyA.Awake && bodyA.BodyType != BodyType.Static)
            return true;

        if (bodyB.Awake && bodyB.BodyType != BodyType.Static)
            return true;

        return false;
    }

    /// <summary>
    /// Marks a contact for filtering and puts it on the collide queue. A settled pair is otherwise
    /// off that queue, and <see cref="CollideContacts"/> only reads <see cref="ContactFlags.Filter"/> there.
    /// </summary>
    internal void FlagContactForFilter(Contact contact)
    {
        contact.Flags |= ContactFlags.Filter;
        EnqueueCollide(contact);
    }

    internal void EnqueueCollide(Contact contact)
    {
        if (contact.QueuedForCollide)
            return;

        if ((contact.Flags & (ContactFlags.Deleting | ContactFlags.Deleted)) != 0)
            return;

        contact.QueuedForCollide = true;
        contact.CollidePrev = _collideTail;
        contact.CollideNext = null;

        if (_collideTail != null)
            _collideTail.CollideNext = contact;
        else
            _collideHead = contact;

        _collideTail = contact;
        _collideCount++;
    }

    private void DequeueCollide(Contact contact)
    {
        if (!contact.QueuedForCollide)
            return;

        var prev = contact.CollidePrev;
        var next = contact.CollideNext;

        if (prev != null)
            prev.CollideNext = next;
        else
            _collideHead = next;

        if (next != null)
            next.CollidePrev = prev;
        else
            _collideTail = prev;

        contact.CollidePrev = null;
        contact.CollideNext = null;
        contact.QueuedForCollide = false;
        _collideCount--;
    }

    private void FinishQueuedContact(Contact contact)
    {
        contact.Generated = true;

        if (!ContactNeedsCollide(contact))
            DequeueCollide(contact);
    }

    private void RefreshQueuedContacts(PhysicsComponent body)
    {
        var node = body.Contacts.First;

        while (node != null)
        {
            var queued = node.Value;
            node = node.Next;

            if (ContactNeedsCollide(queued))
                EnqueueCollide(queued);
            else
                DequeueCollide(queued);
        }
    }

    private void SyncIslandContact(Contact contact)
    {
        var hardTouching = contact.Enabled
                           && contact.IsTouching
                           && contact.FixtureA?.Hard == true
                           && contact.FixtureB?.Hard == true
                           && contact.BodyA != null
                           && contact.BodyB != null
                           && (contact.Flags & (ContactFlags.Deleting | ContactFlags.Deleted)) == 0;

        if (hardTouching)
            LinkIslandContact(contact);
        else
            UnlinkIslandContact(contact);
    }

    private static void LinkIslandContact(Contact contact)
    {
        if (contact.InIslandGraph)
            return;

        contact.InIslandGraph = true;
        LinkIslandSide(contact.BodyA!, contact, true);
        LinkIslandSide(contact.BodyB!, contact, false);
    }

    private static void LinkIslandSide(PhysicsComponent body, Contact contact, bool sideA)
    {
        var head = body.IslandContactHead;

        if (sideA)
        {
            contact.IslandPrevA = null;
            contact.IslandNextA = head;
        }
        else
        {
            contact.IslandPrevB = null;
            contact.IslandNextB = head;
        }

        if (head != null)
        {
            if (head.BodyA == body)
                head.IslandPrevA = contact;
            else
                head.IslandPrevB = contact;
        }

        body.IslandContactHead = contact;
    }

    private static void UnlinkIslandContact(Contact contact)
    {
        if (!contact.InIslandGraph)
            return;

        UnlinkIslandSide(contact.BodyA, contact, true);
        UnlinkIslandSide(contact.BodyB, contact, false);
        contact.InIslandGraph = false;
        contact.IslandNextA = null;
        contact.IslandPrevA = null;
        contact.IslandNextB = null;
        contact.IslandPrevB = null;
    }

    private static void UnlinkIslandSide(PhysicsComponent? body, Contact contact, bool sideA)
    {
        if (body == null)
            return;

        var next = sideA ? contact.IslandNextA : contact.IslandNextB;
        var prev = sideA ? contact.IslandPrevA : contact.IslandPrevB;

        if (prev != null)
        {
            if (prev.BodyA == body)
                prev.IslandNextA = next;
            else
                prev.IslandNextB = next;
        }
        else if (body.IslandContactHead == contact)
        {
            body.IslandContactHead = next;
        }

        if (next == null)
            return;

        if (next.BodyA == body)
            next.IslandPrevA = prev;
        else
            next.IslandPrevB = prev;
    }
}
