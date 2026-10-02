using System;
#if !ROBUST_ANALYZERS_TEST
using JetBrains.Annotations;
#endif

namespace Robust.Shared.IoC
{
    /// <summary>
    /// Specifies that the field this is applied to is a dependency,
    /// which will be resolved by <see cref="IoCManager" /> when the containing class is instantiated.
    /// </summary>
    /// <remarks>
    /// <para>
    /// The dependency is resolved as if <see cref="IoCManager.Resolve{T}()" /> were to be called,
    /// but it avoids circular references and init order issues due to internal code in the <see cref="IoCManager" />.
    /// </para>
    /// <para>
    /// If you would like to run code after the dependencies have been injected, use <see cref="IPostInjectInit" />
    /// </para>
    /// </remarks>
    [AttributeUsage(AttributeTargets.Field)]
#if !ROBUST_ANALYZERS_TEST
    [MeansImplicitUse(ImplicitUseKindFlags.Assign)]
#endif
    public sealed class DependencyAttribute : Attribute
    {
    }
}
