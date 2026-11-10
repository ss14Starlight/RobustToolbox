using System;
using System.Buffers;
using System.Numerics;
using System.Threading;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Collision.Shapes;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Physics.Dynamics.Contacts;

namespace Robust.Shared.Physics.Systems;

public abstract partial class SharedPhysicsSystem
{
    private const float ClippingRecoveryFieldSize = 0.5f;
    private const float ClippingRecoveryMaxFieldSize = 1f;
    /// <summary>
    /// Deeper than this, a body does not count as being at a safe position.
    /// </summary>
    private const float ClippingRecoveryUnsafeSeparation = -3f * PhysicsConstants.LinearSlop;

    /// <summary>
    /// Deeper than this, a body counts as stuck and may be moved out. It is much deeper than
    /// <see cref="ClippingRecoveryUnsafeSeparation"/> because a body resting under load sinks a few centimeters,
    /// and moving it out every time would keep it awake forever.
    /// </summary>
    private const float ClippingRecoveryStuckSeparation = -0.1f;

    /// <summary>
    /// Wait after a search that found no free position before searching for the same body again.
    /// </summary>
    private const float ClippingRecoveryRetryDelay = 5f;

    private const float ClippingRecoveryMaxPush = 0.5f;
    private const float PersistentRecoveryMaxGravityAlignment = 0.5f;

    /// <summary>
    /// Counts server solves, so safe position tracking can tell whether a body took part in the previous one.
    /// </summary>
    private uint _clippingRecoverySolveIndex;

    /// <summary>
    /// Total searches that found no free position. Islands are solved in parallel, so it is updated atomically.
    /// </summary>
    internal int ClippingRecoveryFailedSearches;

    private bool RecoverClippedBodies(
        in SolverData data,
        in IslandData island,
        ContactPositionConstraint[] positionConstraints,
        Vector2[] positions,
        float[] angles,
        Vector2[] linearVelocities,
        float[] angularVelocities,
        bool contactsOkay,
        bool prediction)
    {
        var bodyCount = island.Bodies.Count;
        var offset = island.Offset;
        var moved = !contactsOkay &&
                    RecoverClippedBodiesSimple(in data,
                        in island,
                        positionConstraints,
                        positions,
                        angles,
                        linearVelocities);

        for (var i = 0; i < bodyCount; i++)
        {
            var body = island.Bodies[i].Comp1;
            if (IsClippingRecoveryBody(body))
                body.ClippingRecoveryActive = false;
        }

        for (var i = 0; i < island.Contacts.Count; i++)
        {
            ref readonly var constraint = ref positionConstraints[i];
            var indexA = constraint.IndexA;
            var indexB = constraint.IndexB;
            var bodyA = island.Bodies[indexA].Comp1;
            var bodyB = island.Bodies[indexB].Comp1;

            int clippedIndex;
            if (IsClippingRecoveryBody(bodyA) && bodyB.BodyType == BodyType.Static)
                clippedIndex = indexA;
            else if (IsClippingRecoveryBody(bodyB) && bodyA.BodyType == BodyType.Static)
                clippedIndex = indexB;
            else
                continue;

            var body = island.Bodies[clippedIndex].Comp1;
            if (body.InvMass <= 0f || !body.CanCollide)
                continue;

            var separation = GetContactMinSeparation(in constraint, positions, angles, out _);
            if (separation >= ClippingRecoveryUnsafeSeparation)
                continue;

            if (!prediction)
                body.ClippingRecoveryPenetrating = true;

            if (separation >= ClippingRecoveryStuckSeparation ||
                !CanStartClippingRecovery(in data,
                    body,
                    linearVelocities[offset + clippedIndex],
                    angularVelocities[offset + clippedIndex],
                    prediction))
            {
                continue;
            }

            body.ClippingRecoveryActive = true;
        }

        for (var i = 0; i < bodyCount; i++)
        {
            var body = island.Bodies[i].Comp1;
            if (!IsClippingRecoveryBody(body))
                continue;

            if (!body.ClippingRecoveryActive)
            {
                // A pending retry cooldown (negative time) survives while the body is still clipped,
                // so briefly failing the start conditions does not allow an immediate new search.
                if (!body.ClippingRecoveryPenetrating || body.ClippingRecoveryTime > 0f)
                    body.ClippingRecoveryTime = 0f;
                continue;
            }

            // Positive time counts how long a controller has been stuck. Dynamic bodies already had to settle for
            // the sleep interval to get here, so they need no extra delay. Negative time is a retry cooldown.
            var stuckDelay = MathF.Max(0f, _cfg.GetCVar(CVars.PhysicsClippingRecoveryStuckDelay));
            var startDelay = body.BodyType == BodyType.Dynamic ? 0f : stuckDelay;
            body.ClippingRecoveryTime = MathF.Min(startDelay, body.ClippingRecoveryTime + data.FrameTime);
            if (body.ClippingRecoveryTime < startDelay)
                continue;

            var bodyEnt = island.Bodies[i];
            var mapId = bodyEnt.Comp2.MapID;
            if (mapId == MapId.Nullspace ||
                !_fixturesQuery.TryGetComponent(bodyEnt.Owner, out var fixtures))
            {
                body.ClippingRecoveryTime = 0f;
                continue;
            }

            if (!TryFindClippingRecoveryPosition(
                    mapId,
                    body,
                    fixtures,
                    positions[i],
                    angles[i],
                    out var target))
            {
                // A clipped body can't sleep (its island never counts as position solved), so without a
                // cooldown a body buried in rock would run the full search every tick.
                body.ClippingRecoveryTime = -MathF.Max(stuckDelay, ClippingRecoveryRetryDelay);
                Interlocked.Increment(ref ClippingRecoveryFailedSearches);
                LogManager.GetSawmill("clipping_recovery").Debug(
                    "No free position found for clipped {EntityName} ({EntityUid})",
                    MetaData(bodyEnt.Owner).EntityName,
                    bodyEnt.Owner);
                continue;
            }

            body.ClippingRecoveryTime = 0f;

            // Move in one step to a position verified free of static geometry, so the body never spends
            // ticks inside walls with its static contacts ignored.
            positions[i] = target;
            linearVelocities[offset + i] = Vector2.Zero;
            angularVelocities[offset + i] = 0f;
            SetSleepTime(body, 0f);
            moved = true;
        }

        return moved;
    }

    private void TrackClippingRecoverySafePosition(PhysicsComponent body, TransformComponent xform)
    {
        if (!IsClippingRecoveryBody(body))
            return;

        if (body.ClippingRecoveryCandidateSolve != _clippingRecoverySolveIndex - 1)
        {
            // The body skipped solves (slept, sat in a container, was carried or moved through disposals),
            // so the remembered position may be on the other side of a wall from where it is now.
            body.ClippingRecoverySafeParent = EntityUid.Invalid;
        }
        else if (!body.ClippingRecoveryPenetrating)
        {
            // The candidate is where the previous solve started; it only becomes safe if that solve ended unclipped.
            // Using the previous start rather than the current position ignores teleports straight into walls.
            body.ClippingRecoverySafeParent = body.ClippingRecoveryCandidateParent;
            body.ClippingRecoverySafePosition = body.ClippingRecoveryCandidatePosition;
        }

        body.ClippingRecoveryCandidateSolve = _clippingRecoverySolveIndex;
        body.ClippingRecoveryCandidateParent = xform.ParentUid;
        body.ClippingRecoveryCandidatePosition = xform.LocalPosition;
        body.ClippingRecoveryPenetrating = false;
    }

    private bool TryGetClippingRecoverySafePosition(
        PhysicsComponent body,
        MapId mapId,
        float angle,
        out Vector2 safePosition)
    {
        safePosition = default;
        if (!body.ClippingRecoverySafeParent.IsValid() ||
            !XformQuery.TryGetComponent(body.ClippingRecoverySafeParent, out var parentXform) ||
            parentXform.MapID != mapId)
        {
            return false;
        }

        var origin = Vector2.Transform(body.ClippingRecoverySafePosition, _transform.GetWorldMatrix(parentXform));
        safePosition = origin + Physics.Transform.Mul(new Transform(angle).Quaternion2D, body.LocalCenter);
        return true;
    }

    private bool RecoverClippedBodiesSimple(
        in SolverData data,
        in IslandData island,
        ContactPositionConstraint[] positionConstraints,
        Vector2[] positions,
        float[] angles,
        Vector2[] linearVelocities)
    {
        var bodyCount = island.Bodies.Count;
        var offset = island.Offset;
        var minRecoverySeparation = -MathF.Max(
            3.0f * PhysicsConstants.LinearSlop,
            data.MaxLinearCorrection);
        var minPersistentRecoverySeparation = -3.0f * PhysicsConstants.LinearSlop;
        var gravity = Gravity;
        var hasGravity = gravity.LengthSquared() > 0f;
        var gravityDirection = hasGravity ? Vector2.Normalize(gravity) : Vector2.Zero;

        float[]? deepest = null;
        Vector2[]? pushDirs = null;
        var moved = false;

        try
        {
            for (var i = 0; i < island.Contacts.Count; i++)
            {
                ref readonly var constraint = ref positionConstraints[i];
                var indexA = constraint.IndexA;
                var indexB = constraint.IndexB;
                var bodyA = island.Bodies[indexA].Comp1;
                var bodyB = island.Bodies[indexB].Comp1;

                int clippedIndex;
                if (bodyA.BodyType == BodyType.Dynamic && bodyB.BodyType == BodyType.Static)
                    clippedIndex = indexA;
                else if (bodyB.BodyType == BodyType.Dynamic && bodyA.BodyType == BodyType.Static)
                    clippedIndex = indexB;
                else
                    continue;

                var body = island.Bodies[clippedIndex].Comp1;
                if (body.InvMass <= 0f || !body.CanCollide)
                    continue;

                var separation = GetContactMinSeparation(in constraint, positions, angles, out var contactNormal);
                if (contactNormal == Vector2.Zero)
                    continue;

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

                if (deepest == null)
                {
                    deepest = ArrayPool<float>.Shared.Rent(bodyCount);
                    pushDirs = ArrayPool<Vector2>.Shared.Rent(bodyCount);
                    Array.Clear(deepest, 0, bodyCount);
                }

                if (separation >= deepest[clippedIndex])
                    continue;

                deepest[clippedIndex] = separation;
                pushDirs![clippedIndex] = clippedIndex == indexA ? -contactNormal : contactNormal;
            }

            if (deepest == null)
                return false;

            for (var i = 0; i < bodyCount; i++)
            {
                var separation = deepest[i];
                if (separation >= 0f)
                    continue;

                var position = positions[i];
                if (!float.IsFinite(position.X) || !float.IsFinite(position.Y))
                    continue;

                var pushDir = pushDirs![i];

                ref var velocity = ref linearVelocities[offset + i];
                var intoWall = Vector2.Dot(velocity, pushDir);
                if (intoWall < 0f)
                    velocity -= pushDir * intoWall;

                var depth = -separation + PhysicsConstants.LinearSlop;
                if (depth > ClippingRecoveryMaxPush)
                    continue;

                positions[i] = position + pushDir * depth;
                moved = true;
            }

            return moved;
        }
        finally
        {
            if (deepest != null)
            {
                ArrayPool<float>.Shared.Return(deepest);
                ArrayPool<Vector2>.Shared.Return(pushDirs!);
            }
        }
    }

    private bool TryFindClippingRecoveryPosition(
        MapId mapId,
        PhysicsComponent body,
        FixturesComponent fixtures,
        Vector2 position,
        float angle,
        out Vector2 target)
    {
        var maxDistance = MathF.Max(0f, _cfg.GetCVar(CVars.PhysicsClippingRecoveryMaxDistance));
        var preferredDirection = Vector2.Zero;

        if (TryGetClippingRecoverySafePosition(body, mapId, angle, out var safePosition))
        {
            var toSafe = safePosition - position;
            var distanceSquared = toSafe.LengthSquared();
            if (distanceSquared <= maxDistance * maxDistance)
            {
                if (!IsPositionColliding(mapId, body, fixtures, safePosition, angle))
                {
                    target = safePosition;
                    return true;
                }

                // Something now occupies the safe position; still prefer to come out on that side.
                if (distanceSquared > float.Epsilon)
                    preferredDirection = toSafe / MathF.Sqrt(distanceSquared);
            }
        }

        var fieldSize = GetClippingRecoverySampleSpacing(fixtures, angle);
        var collisionQueries = 0;
        var fullScanDistance = MathF.Min(
            maxDistance,
            MathF.Max(0f, _cfg.GetCVar(CVars.PhysicsClippingRecoveryFullScanDistance)));
        var maxOuterRingSamples = Math.Max(0, _cfg.GetCVar(CVars.PhysicsClippingRecoveryMaxOuterRingSamples));
        var maxCollisionQueries = Math.Max(0, _cfg.GetCVar(CVars.PhysicsClippingRecoveryMaxCollisionQueries));
        var offset = FindClippingRecoveryOffset(
            mapId,
            body,
            fixtures,
            position,
            angle,
            preferredDirection,
            fieldSize,
            maxDistance,
            fullScanDistance,
            maxOuterRingSamples,
            maxCollisionQueries,
            ref collisionQueries);

        target = position + offset;
        return offset != Vector2.Zero;
    }

    private static float GetClippingRecoverySampleSpacing(FixturesComponent fixtures, float angle)
    {
        var transform = new Transform(angle);
        var bounds = default(Box2);
        var hasBounds = false;

        foreach (var fixture in fixtures.Fixtures.Values)
        {
            if (!fixture.Hard || (fixture.CollisionLayer == 0 && fixture.CollisionMask == 0))
                continue;

            for (var childIndex = 0; childIndex < fixture.Shape.ChildCount; childIndex++)
            {
                var fixtureBounds = fixture.Shape.ComputeAABB(transform, childIndex);
                if (hasBounds)
                {
                    bounds = bounds.Union(fixtureBounds);
                }
                else
                {
                    bounds = fixtureBounds;
                    hasBounds = true;
                }
            }
        }

        if (!hasBounds)
            return ClippingRecoveryFieldSize;

        var size = bounds.TopRight - bounds.BottomLeft;
        var objectSize = MathF.Max(size.X, size.Y);
        return MathF.Max(
            ClippingRecoveryFieldSize,
            MathF.Min(ClippingRecoveryMaxFieldSize, objectSize * 0.5f));
    }

    /// <summary>
    /// Searches square rings around the body for an offset free of static geometry. Without a preferred direction the
    /// first free sample wins; with one, the free sample of the nearest ring that best matches it wins.
    /// </summary>
    private Vector2 FindClippingRecoveryOffset(
        MapId mapId,
        PhysicsComponent body,
        FixturesComponent fixtures,
        Vector2 position,
        float angle,
        Vector2 preferredDirection,
        float fieldSize,
        float maxDistance,
        float fullScanDistance,
        int maxOuterRingSamples,
        int maxCollisionQueries,
        ref int collisionQueries)
    {
        var maxRing = (int)MathF.Ceiling(maxDistance / fieldSize);
        var maxDistanceSquared = maxDistance * maxDistance;
        for (var ring = 1; ring <= maxRing; ring++)
        {
            var ringSampleCount = 8 * ring;
            var validSampleCount = 0;
            for (var sampleIndex = 0; sampleIndex < ringSampleCount; sampleIndex++)
            {
                var offset = GetClippingRecoveryRingOffset(ring, sampleIndex, fieldSize);
                if (offset.LengthSquared() <= maxDistanceSquared)
                    validSampleCount++;
            }

            if (validSampleCount == 0)
                continue;

            var remainingQueries = maxCollisionQueries - collisionQueries;
            var ringSampleLimit = ring * fieldSize <= fullScanDistance
                ? validSampleCount
                : maxOuterRingSamples;
            var samplesThisRing = Math.Min(Math.Min(validSampleCount, ringSampleLimit), remainingQueries);
            if (samplesThisRing == 0)
                return Vector2.Zero;

            var validSampleIndex = 0;
            var sampledIndex = 0;
            var bestOffset = Vector2.Zero;
            var bestAlignment = float.NegativeInfinity;
            for (var sampleIndex = 0; sampleIndex < ringSampleCount; sampleIndex++)
            {
                var offset = GetClippingRecoveryRingOffset(ring, sampleIndex, fieldSize);
                var distanceSquared = offset.LengthSquared();
                if (distanceSquared > maxDistanceSquared)
                    continue;

                var shouldCheck = samplesThisRing == validSampleCount ||
                                  validSampleIndex == sampledIndex * validSampleCount / samplesThisRing;
                validSampleIndex++;
                if (!shouldCheck)
                    continue;

                sampledIndex++;

                var alignment = Vector2.Dot(offset, preferredDirection) / MathF.Sqrt(distanceSquared);
                if (alignment > bestAlignment)
                {
                    collisionQueries++;
                    if (!IsPositionColliding(mapId, body, fixtures, position + offset, angle))
                    {
                        if (preferredDirection == Vector2.Zero)
                            return offset;

                        bestOffset = offset;
                        bestAlignment = alignment;
                    }
                }

                if (sampledIndex == samplesThisRing)
                    break;
            }

            if (bestOffset != Vector2.Zero)
                return bestOffset;

            if (collisionQueries >= maxCollisionQueries)
                return Vector2.Zero;
        }

        return Vector2.Zero;
    }

    private static Vector2 GetClippingRecoveryRingOffset(int ring, int sampleIndex, float fieldSize)
    {
        var ringSampleCount = 8 * ring;
        // Start at +Y so floor contacts can recover upward on the first successful sample.
        var perimeterSampleIndex = (sampleIndex + 5 * ring) % ringSampleCount;
        var sideLength = ring * 2;
        var side = perimeterSampleIndex / sideLength;
        var index = perimeterSampleIndex % sideLength;
        var offset = side switch
        {
            0 => new Vector2(-ring + index, -ring),
            1 => new Vector2(ring, -ring + index),
            2 => new Vector2(ring - index, ring),
            _ => new Vector2(-ring, ring - index),
        };
        return offset * fieldSize;
    }

    private bool IsPositionColliding(
        MapId mapId,
        PhysicsComponent body,
        FixturesComponent fixtures,
        Vector2 position,
        float angle)
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
                    Flags = QueryFlags.Static
                });

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

    private static bool IsClippingRecoveryBody(PhysicsComponent body) =>
        body.BodyType is BodyType.Dynamic or BodyType.KinematicController;

    private static bool CanStartClippingRecovery(
        in SolverData data,
        PhysicsComponent body,
        Vector2 linearVelocity,
        float angularVelocity,
        bool prediction)
    {
        return !prediction &&
               (body.BodyType == BodyType.KinematicController ||
                data.SleepAllowed &&
                body.SleepingAllowed &&
                body.SleepTime + data.FrameTime >= data.TimeToSleep &&
                linearVelocity.LengthSquared() <= data.LinTolSqr &&
                angularVelocity * angularVelocity <= data.AngTolSqr);
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

    /// <summary>
    /// Allows a fully resting dynamic body to sleep when its only constraints are unsolved hard
    /// contacts against static geometry. Dynamic, kinematic, and joint-connected contacts retain
    /// the normal <see cref="IslandData.PositionSolved"/> requirement.
    /// </summary>
    private static bool CanSleepWithUnsolvedStaticContacts(in IslandData island)
    {
        if (island.Joints.Count != 0 || island.Contacts.Count == 0)
            return false;

        foreach (var contact in island.Contacts)
        {
            if (!contact.Enabled ||
                !contact.IsTouching ||
                contact.FixtureA?.Hard != true ||
                contact.FixtureB?.Hard != true ||
                contact.BodyA is not { } bodyA ||
                contact.BodyB is not { } bodyB)
            {
                return false;
            }

            var bodyAStatic = bodyA.BodyType == BodyType.Static;
            var bodyBStatic = bodyB.BodyType == BodyType.Static;

            // Require exactly one static and one ordinary dynamic body.
            if (bodyAStatic == bodyBStatic)
                return false;

            var dynamicBody = bodyAStatic ? bodyB : bodyA;
            if (dynamicBody.BodyType != BodyType.Dynamic)
                return false;
        }

        return true;
    }

    /// <summary>
    /// True when this step's velocity and position correction are both under the sleep tolerances.
    /// Position correction never writes <see cref="PhysicsComponent.LinearVelocity"/>, so a body
    /// being pushed out of a wall would otherwise accumulate sleep time while it is still moving.
    /// </summary>
    private static bool RestingThisStep(
        PhysicsComponent body,
        TransformComponent xform,
        Vector2 positionDelta,
        float angleDelta,
        in SolverData data)
    {
        if (!body.SleepingAllowed)
            return false;

        if (body.AngularVelocity * body.AngularVelocity > data.AngTolSqr ||
            Vector2.Dot(body.LinearVelocity, body.LinearVelocity) > data.LinTolSqr)
            return false;

        if (float.IsNaN(positionDelta.X) || float.IsNaN(positionDelta.Y) || float.IsNaN(angleDelta))
            return false;

        // FinalisePositions stores the transform-origin delta, but the solver integrates the
        // center of mass. Remove origin motion caused only by rotation around an offset center.
        var oldRotation = new Quaternion2D(xform._localRotation);
        var newRotation = new Quaternion2D(xform._localRotation + angleDelta);
        var centerDelta = positionDelta
                          + Physics.Transform.Mul(newRotation, body.LocalCenter)
                          - Physics.Transform.Mul(oldRotation, body.LocalCenter);

        var lin = centerDelta * data.InvDt;
        if (Vector2.Dot(lin, lin) > data.LinTolSqr)
            return false;

        var ang = angleDelta * data.InvDt;
        if (ang * ang > data.AngTolSqr)
            return false;

        return true;
    }

    private readonly struct AnyFixtureOverlapCallback : IFixtureQueryCallback<bool>
    {
        public bool Invoke(ref bool state, in FixtureProxy fixture)
        {
            state = true;
            return false;
        }
    }
}
