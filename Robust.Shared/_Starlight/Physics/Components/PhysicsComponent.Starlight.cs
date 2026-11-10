using System.Numerics;
using Robust.Shared.Physics.Dynamics.Contacts;
using Transform = Robust.Shared.Physics.Transform;
using Robust.Shared.GameObjects;

namespace Robust.Shared.Physics.Components;

public sealed partial class PhysicsComponent
{
    internal bool ClippingRecoveryActive;
    internal Vector2 ClippingRecoveryDirection;

    // Starlight: per-step transform cache and the head of this body's island-contact list.
    internal uint TransformStamp;
    internal Transform TransformCache;
    internal Contact? IslandContactHead;

    /// <summary>
    /// Positive: how long the body has been stuck. Negative: cooldown left after a search that found no free position.
    /// </summary>
    internal float ClippingRecoveryTime;

    /// <summary>
    /// Whether the last server solve left this body clipped into static geometry.
    /// </summary>
    internal bool ClippingRecoveryPenetrating;

    /// <summary>
    /// Parent-local position at the start of the last solve. Promoted to the safe position if that solve ended unclipped
    /// and the body took part in the very next solve as well.
    /// </summary>
    internal uint ClippingRecoveryCandidateSolve;
    internal EntityUid ClippingRecoveryCandidateParent;
    internal Vector2 ClippingRecoveryCandidatePosition;

    /// <summary>
    /// Last known parent-local position where the body was not clipped. Recovery returns the body here so it
    /// leaves a wall on the side it came from.
    /// </summary>
    internal EntityUid ClippingRecoverySafeParent;
    internal Vector2 ClippingRecoverySafePosition;
}
