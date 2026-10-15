using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.Serialization.Markdown;
using Robust.Shared.Serialization.Markdown.Mapping;
using Robust.Shared.Serialization.Markdown.Sequence;
using Robust.Shared.Serialization.Markdown.Value;

namespace Robust.UnitTesting.Shared.Serialization;

[TestFixture]
internal sealed partial class SequenceRemoveInheritanceTest : OurSerializationTest
{
    [DataDefinition]
    private sealed partial class RemoveTestClass
    {
        [DataField("list")]
        public List<string> List = new();

        [DataField("alwaysList"), AlwaysPushInheritance]
        public List<string> AlwaysList = new();
    }

    private static ValueDataNode Removal(string value) => new(value) { Tag = "!Remove" };

    private static ValueDataNode Entry(string value) => new(value);

    private static MappingDataNode Map(string field, params DataNode[] entries)
    {
        return new MappingDataNode { { field, new SequenceDataNode(entries) } };
    }

    private RemoveTestClass Compose(MappingDataNode child, params MappingDataNode[] parents)
    {
        var mapping = Serialization.PushComposition<RemoveTestClass, MappingDataNode>(parents, child);
        return Serialization.Read<RemoveTestClass>(mapping, notNullableOverride: true);
    }

    [Test]
    public void ChildWithoutRemoveStillReplacesParent()
    {
        var val = Compose(Map("list", Entry("C")), Map("list", Entry("A"), Entry("B")));

        Assert.That(val.List, Is.EqualTo(new[] { "C" }));
    }

    [Test]
    public void ChildRemovesFromParent()
    {
        var val = Compose(
            Map("list", Removal("A"), Entry("C")),
            Map("list", Entry("A"), Entry("B")));

        Assert.That(val.List, Is.EqualTo(new[] { "B", "C" }));
    }

    [Test]
    public void LaterParentRemovesFromEarlierParent()
    {
        var val = Compose(
            new MappingDataNode(),
            Map("list", Entry("A"), Entry("B"), Entry("C")),
            Map("list", Removal("B")));

        Assert.That(val.List, Is.EqualTo(new[] { "A", "C" }));
    }

    [Test]
    public void EarlierParentRemovesFromLaterParent()
    {
        var val = Compose(
            new MappingDataNode(),
            Map("list", Removal("B")),
            Map("list", Entry("A"), Entry("B"), Entry("C")));

        Assert.That(val.List, Is.EqualTo(new[] { "A", "C" }));
    }

    [Test]
    public void RemovesWaitForAParentToApplyTo()
    {
        var val = Compose(
            new MappingDataNode(),
            Map("list", Removal("A")),
            Map("list", Removal("B")),
            Map("list", Entry("A"), Entry("B"), Entry("C")));

        Assert.That(val.List, Is.EqualTo(new[] { "C" }));
    }

    [Test]
    public void RemovingMissingEntryDoesNothing()
    {
        var val = Compose(Map("list", Removal("X")), Map("list", Entry("A")));

        Assert.That(val.List, Is.EqualTo(new[] { "A" }));
    }

    [Test]
    public void AlwaysPushWithoutRemoveStillAppends()
    {
        var val = Compose(Map("alwaysList", Entry("C")), Map("alwaysList", Entry("A"), Entry("B")));

        Assert.That(val.AlwaysList, Is.EqualTo(new[] { "C", "A", "B" }));
    }

    [Test]
    public void AlwaysPushChildRemovesFromParent()
    {
        var val = Compose(
            Map("alwaysList", Removal("A"), Entry("C")),
            Map("alwaysList", Entry("A"), Entry("B")));

        Assert.That(val.AlwaysList, Is.EqualTo(new[] { "B", "C" }));
    }

    [Test]
    public void UnappliedRemoveFailsValidation()
    {
        var errors = Serialization.ValidateNode<RemoveTestClass>(Map("list", Removal("A"), Entry("B")))
            .GetErrors()
            .ToList();

        Assert.That(errors, Has.Count.EqualTo(1));
        Assert.That(errors[0].AlwaysRelevant, Is.False);
    }

    [Test]
    public void AppliedRemovePassesValidation()
    {
        var mapping = Serialization.PushComposition<RemoveTestClass, MappingDataNode>(
            new[] { Map("list", Entry("A"), Entry("B")) },
            Map("list", Removal("A")));

        Assert.That(Serialization.ValidateNode<RemoveTestClass>(mapping).GetErrors(), Is.Empty);
    }
}
