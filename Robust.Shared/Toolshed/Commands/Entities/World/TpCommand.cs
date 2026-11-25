using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;

namespace Robust.Shared.Toolshed.Commands.Entities.World;

[ToolshedCommand]
internal sealed partial class TpCommand : ToolshedCommand // Starlight edit
{
    private SharedTransformSystem? _xform;

    // TODO TOOLSHED
    // add EntityCoordinates parser

    [CommandImplementation("coords")]
    public EntityUid TpCoords([PipedArgument] EntityUid teleporter, [CommandArgument(unparseable:true)] EntityCoordinates target)
    {
        _xform ??= GetSys<SharedTransformSystem>();
        _xform.SetCoordinates(teleporter, target);
        return teleporter;
    }

    [CommandImplementation("coords")]
    public IEnumerable<EntityUid> TpCoords([PipedArgument] IEnumerable<EntityUid> teleporters, [CommandArgument(unparseable:true)] EntityCoordinates target)
        => teleporters.Select(x => TpCoords(x, target));

    [CommandImplementation("to")]
    public EntityUid TpTo(IInvocationContext ctx, [PipedArgument] EntityUid teleporter, [CommandArgument(typeof(EntityOrSessionParser))] EntityUid target) // Starlight edit
    {
        // Starlight begin
        if (!CanTeleportTo(ctx, teleporter, target))
            return teleporter;
        // Starlight end

        _xform ??= GetSys<SharedTransformSystem>();
        _xform.SetCoordinates(teleporter, Transform(target).Coordinates);
        return teleporter;
    }

    [CommandImplementation("to")]
    // Starlight begin
    public IEnumerable<EntityUid> TpTo(
        IInvocationContext ctx,
        [PipedArgument] IEnumerable<EntityUid> teleporters,
        [CommandArgument(typeof(EntityOrSessionParser))] EntityUid target) =>
        [.. teleporters.Select(x => TpTo(ctx, x, target))];
    // Starlight end

    [CommandImplementation("into")]
    public EntityUid TpInto([PipedArgument] EntityUid teleporter, EntityUid target)
    {
        _xform ??= GetSys<SharedTransformSystem>();
        _xform.SetCoordinates(teleporter, new EntityCoordinates(target, Vector2.Zero));
        return teleporter;
    }

    [CommandImplementation("into")]
    public IEnumerable<EntityUid> TpInto([PipedArgument] IEnumerable<EntityUid> teleporters, EntityUid target)
        => teleporters.Select(x => TpInto(x, target));
}
