using System.Linq;
using Robust.Shared.Console;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Map.Components;
using Robust.Shared.Player;
using Robust.Shared.Toolshed.Errors;
using Robust.Shared.Toolshed.Syntax;
using Robust.Shared.Toolshed.TypeParsers;
using Robust.Shared.Utility;

// ReSharper disable CheckNamespace
namespace Robust.Shared.Toolshed.Commands.Entities.World;

internal sealed partial class TpCommand
{
    [CommandImplementation("to")]
    public EntityUid TpTo(IInvocationContext ctx, [CommandArgument(typeof(EntityOrSessionParser))] EntityUid target)
    {
        if (ctx.Session is null)
        {
            ctx.ReportError(new NotForServerConsoleError());
            return default;
        }

        if (ctx.Session.AttachedEntity is { } self) return TpTo(ctx, self, target);
        ctx.ReportError(new SessionHasNoEntityError(ctx.Session));
        return default;
    }

    private bool CanTeleportTo(IInvocationContext ctx, EntityUid teleporter, EntityUid target)
    {
        var ev = new ToolshedTeleportAttemptEvent(ctx.Session, teleporter, target);
        EntityManager.EventBus.RaiseEvent(EventSource.Local, ref ev);

        if (!ev.Cancelled)
            return true;

        ctx.WriteMarkup($"[color=red]{ev.Reason ?? "You can't teleport that there."}[/color]");
        return false;
    }
}

[ByRefEvent]
public record struct ToolshedTeleportAttemptEvent(ICommonSession? User, EntityUid Teleporter, EntityUid Target)
{
    public bool Cancelled;
    public string? Reason;
}

internal sealed partial class EntityOrSessionParser : CustomTypeParser<EntityUid>
{
    [Dependency] private IEntityManager _entMan = null!;
    [Dependency] private ISharedPlayerManager _player = null!;

    public override bool TryParse(ParserContext ctx, out EntityUid result)
    {
        var restore = ctx.Save();

        if (EntityTypeParser.TryParseEntity(_entMan, ctx, out result)
            && _entMan.EntityExists(result)
            && !_entMan.HasComponent<MapComponent>(result))
            return true;

        ctx.Restore(restore);
        ctx.Error = null;

        var start = ctx.Index;
        var word = ctx.GetWord();
        if (word == null)
        {
            ctx.Error = new OutOfInputError();
            return false;
        }

        if (!_player.TryGetSessionByUsername(word, out var session)
            && !(System.Guid.TryParse(word, out var guid) && _player.TryGetSessionById(new(guid), out session)))
        {
            ctx.Error = new InvalidEntity(word);
            ctx.Error.Contextualize(ctx.Input, (start, ctx.Index));
            return false;
        }

        if (session.AttachedEntity is not { } attached)
        {
            ctx.Error = new SessionHasNoEntityError(session);
            ctx.Error.Contextualize(ctx.Input, (start, ctx.Index));
            return false;
        }

        result = attached;
        return true;
    }

    public override CompletionResult TryAutocomplete(ParserContext ctx, CommandArgument? arg)
    {
        var options = CompletionHelper.SessionNames(true, _player).ToList();
        return CompletionResult.FromHintOptions(options, ToolshedCommand.GetArgHint(arg, typeof(NetEntity)));
    }
}
