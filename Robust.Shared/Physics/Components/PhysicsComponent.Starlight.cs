using System.Numerics;
using Robust.Shared.Physics.Dynamics.Contacts;
using Transform = Robust.Shared.Physics.Transform;

namespace Robust.Shared.Physics.Components;

public sealed partial class PhysicsComponent
{
    internal bool ClippingRecoveryActive;
    internal Vector2 ClippingRecoveryDirection;

    // Starlight: per-step transform cache and the head of this body's island-contact list.
    internal uint TransformStamp;
    internal Transform TransformCache;
    internal Contact? IslandContactHead;
    internal float ClippingRecoveryTime;
}
