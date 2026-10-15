using System;
using System.Diagnostics.CodeAnalysis;
using Robust.Shared.Serialization.Markdown;
using Robust.Shared.Serialization.Markdown.Sequence;
using Robust.Shared.Serialization.Markdown.Validation;
using PrototypeManager = Robust.Shared.Prototypes.PrototypeManager;

// ReSharper disable once CheckNamespace
namespace Robust.Shared.Serialization.Manager;

public partial class SerializationManager
{
    /// <summary>
    /// A sequence with <c>!Remove</c> entries edits the one it inherits instead of replacing it, like in partials.
    /// Stock-engine clients don't apply this, so only use it on data the client doesn't read.
    /// </summary>
    private static bool TryCombineSequenceEdits(
        DataNode child,
        DataNode parent,
        [NotNullWhen(true)] out SequenceDataNode? result)
    {
        result = null;
        if (child is not SequenceDataNode childSequence || parent is not SequenceDataNode parentSequence)
            return false;

        var childEdits = HasRemoveEntries(childSequence);
        var parentEdits = HasRemoveEntries(parentSequence);
        if (!childEdits && !parentEdits)
            return false;

        // Nothing to edit yet, so keep both for a later parent.
        if (childEdits && parentEdits)
        {
            result = childSequence.Copy();
            foreach (var entry in parentSequence)
            {
                result.Add(entry.Copy());
            }

            return true;
        }

        SequenceDataNode edits;
        if (childEdits)
        {
            result = parentSequence.Copy();
            edits = childSequence;
        }
        else
        {
            result = childSequence.Copy();
            edits = parentSequence;
        }

        foreach (var entry in edits)
        {
            if (IsRemoveEntry(entry))
            {
                result.Remove(entry);
                continue;
            }

            result.Add(entry.Copy());
        }

        return true;
    }

    /// <summary>
    /// A <c>!Remove</c> left after inheritance had nothing to remove, and would be read as a normal entry.
    /// </summary>
    private static bool TryGetUnappliedRemoveError(DataNode node, [NotNullWhen(true)] out ErrorNode? error)
    {
        if (!IsRemoveEntry(node))
        {
            error = null;
            return false;
        }

        // Not always relevant, so prototype uploads don't fail on it.
        error = new ErrorNode(
            node,
            $"{PrototypeManager.PartialRemoveTag} has nothing to remove: no parent provides this field.",
            alwaysRelevant: false);
        return true;
    }

    private static bool HasRemoveEntries(SequenceDataNode sequence)
    {
        foreach (var entry in sequence)
        {
            if (IsRemoveEntry(entry))
                return true;
        }

        return false;
    }

    private static bool IsRemoveEntry(DataNode node)
    {
        return node.Tag != null && node.Tag.Equals(PrototypeManager.PartialRemoveTag, StringComparison.OrdinalIgnoreCase);
    }
}
