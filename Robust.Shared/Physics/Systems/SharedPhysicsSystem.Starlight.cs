using System;
using System.Buffers;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Physics.Dynamics.Contacts;

namespace Robust.Shared.Physics.Systems;

/*
 * All of this solves the issue of entities getting stuck in walls or close to it forever
 * Ending up causing physics movements and massiv lag until the round ends
 */

public abstract partial class SharedPhysicsSystem
{
    private const float ClippingRecoverySearchStep = 0.25f;
    private const int ClippingRecoverySearchRings = 8;
    private const float ClippingRecoveryFallbackDistance =
        ClippingRecoverySearchStep * (ClippingRecoverySearchRings + 1);
    private const float PersistentRecoveryMaxGravityAlignment = 0.5f;

    private void RecoverClippedBodies(
        in SolverData data,
        in IslandData island,
        ContactPositionConstraint[] positionConstraints,
        Vector2[] positions,
        float[] angles)
    {
        var bodyCount = island.Bodies.Count;
        var minRecoverySeparation = -MathF.Max(
            3.0f * PhysicsConstants.LinearSlop,
            data.MaxLinearCorrection);
        var minPersistentRecoverySeparation = -3.0f * PhysicsConstants.LinearSlop;
        var gravity = Gravity;
        var hasGravity = gravity.LengthSquared() > 0f;
        var gravityDirection = hasGravity ? Vector2.Normalize(gravity) : Vector2.Zero;
        bool[]? attempted = null;

        try
        {
            for (var i = 0; i < island.Contacts.Count; i++)
            {
                ref readonly var constraint = ref positionConstraints[i];
                var indexA = constraint.IndexA;
                var indexB = constraint.IndexB;
                var bodyA = island.Bodies[indexA].Comp1;
                var bodyB = island.Bodies[indexB].Comp1;
                var clippedIndex = bodyA.BodyType == BodyType.Dynamic && bodyB.BodyType == BodyType.Static
                    ? indexA
                    : bodyB.BodyType == BodyType.Dynamic && bodyA.BodyType == BodyType.Static
                        ? indexB
                        : -1;

                if (clippedIndex < 0 || attempted?[clippedIndex] == true)
                    continue;

                var bodyEnt = island.Bodies[clippedIndex];
                var body = bodyEnt.Comp1;
                var separation = GetContactMinSeparation(in constraint, positions, angles, out var contactNormal);
                var sideContact = !hasGravity ||
                                  MathF.Abs(Vector2.Dot(contactNormal, gravityDirection)) <
                                  PersistentRecoveryMaxGravityAlignment;
                var lowMotionForSleepInterval = sideContact
                                                && data.SleepAllowed
                                                && body.SleepingAllowed
                                                && body.SleepTime >= data.TimeToSleep;
                var recoverySeparation = lowMotionForSleepInterval
                    ? minPersistentRecoverySeparation
                    : minRecoverySeparation;

                if (separation >= recoverySeparation)
                    continue;

                if (attempted == null)
                {
                    attempted = ArrayPool<bool>.Shared.Rent(bodyCount);
                    Array.Clear(attempted, 0, bodyCount);
                }

                attempted[clippedIndex] = true;

                if (body.InvMass <= 0f || !body.CanCollide ||
                    !_fixturesQuery.TryGetComponent(bodyEnt.Owner, out var fixtures))
                {
                    continue;
                }

                var xform = bodyEnt.Comp2;
                var mapId = xform.MapID;
                if (mapId == MapId.Nullspace)
                    continue;

                var position = positions[clippedIndex];
                var angle = angles[clippedIndex];
                if (!float.IsFinite(position.X) || !float.IsFinite(position.Y) || !float.IsFinite(angle))
                    continue;

                if (TryFindClosestNonCollidingPosition(
                        bodyEnt.Owner,
                        mapId,
                        body,
                        fixtures,
                        position,
                        angle,
                        out var freePosition))
                {
                    positions[clippedIndex] = freePosition;
                }
                else
                {
                    positions[clippedIndex] = position + Direction.South.ToVec() * ClippingRecoveryFallbackDistance;
                }
            }
        }
        finally
        {
            if (attempted != null)
                ArrayPool<bool>.Shared.Return(attempted);
        }
    }

    private bool TryFindClosestNonCollidingPosition(
        EntityUid bodyUid,
        MapId mapId,
        PhysicsComponent body,
        FixturesComponent fixtures,
        Vector2 position,
        float angle,
        out Vector2 freePosition)
    {
        Func<EntityUid, bool> ignoreBody = uid => uid == bodyUid;

        if (!IsPositionColliding(mapId, body, fixtures, position, angle, ignoreBody))
        {
            freePosition = position;
            return true;
        }

        for (var ring = 1; ring <= ClippingRecoverySearchRings; ring++)
        {
            var distance = ring * ClippingRecoverySearchStep;

            foreach (var direction in DirectionExtensions.AllDirections)
            {
                var candidate = position + direction.ToVec() * distance;
                if (IsPositionColliding(mapId, body, fixtures, candidate, angle, ignoreBody))
                    continue;

                freePosition = candidate;
                return true;
            }
        }

        freePosition = position;
        return false;
    }

    private bool IsPositionColliding(
        MapId mapId,
        PhysicsComponent body,
        FixturesComponent fixtures,
        Vector2 position,
        float angle,
        Func<EntityUid, bool> ignoreBody)
    {
        var bodyTransform = new Transform(angle);
        bodyTransform.Position = position - Physics.Transform.Mul(bodyTransform.Quaternion2D, body.LocalCenter);

        foreach (var fixture in fixtures.Fixtures.Values)
        {
            if (!fixture.Hard || (fixture.CollisionLayer == 0 && fixture.CollisionMask == 0))
                continue;

            var query = new FixtureQueryArgs(
                new QueryFilter
                {
                    LayerBits = fixture.CollisionLayer,
                    MaskBits = fixture.CollisionMask,
                    Flags = QueryFlags.Dynamic | QueryFlags.Static,
                    IsIgnored = ignoreBody
                },
                Approximate: false);

            for (var childIndex = 0; childIndex < fixture.Shape.ChildCount; childIndex++)
            {
                var colliding = false;
                _lookup.ForEachFixtureIntersecting(
                    mapId,
                    fixture.Shape,
                    childIndex,
                    bodyTransform,
                    ref colliding,
                    new AnyFixtureOverlapCallback(),
                    query);

                if (colliding)
                    return true;
            }
        }

        return false;
    }

    private readonly struct AnyFixtureOverlapCallback : IFixtureQueryCallback<bool>
    {
        public bool Invoke(ref bool state, in FixtureProxy fixture)
        {
            state = true;
            return false;
        }
    }

    private static float GetContactMinSeparation(
        in ContactPositionConstraint constraint,
        Vector2[] positions,
        float[] angles,
        out Vector2 contactNormal)
    {
        var indexA = constraint.IndexA;
        var indexB = constraint.IndexB;

        var xfA = new Transform(angles[indexA]);
        var xfB = new Transform(angles[indexB]);
        xfA.Position = positions[indexA] - Physics.Transform.Mul(xfA.Quaternion2D, constraint.LocalCenterA);
        xfB.Position = positions[indexB] - Physics.Transform.Mul(xfB.Quaternion2D, constraint.LocalCenterB);

        var minSeparation = 0.0f;
        contactNormal = Vector2.Zero;
        for (var i = 0; i < constraint.PointCount; i++)
        {
            PositionSolverManifoldInitialize(constraint, i, xfA, xfB, out var normal, out _, out var separation);
            if (separation >= minSeparation)
                continue;

            minSeparation = separation;
            contactNormal = normal;
        }

        return minSeparation;
    }
}
