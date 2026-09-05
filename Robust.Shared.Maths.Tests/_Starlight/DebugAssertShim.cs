using System;
using System.Diagnostics;
using System.Reflection;
using NUnit.Framework;
using Robust.Shared.Utility;

/// <summary>
/// Makes failed <see cref="Debug.Assert"/> calls throw instead of aborting the process.
/// </summary>
[SetUpFixture]
public sealed class DebugAssertShim
{
    [OneTimeSetUp]
    public void InstallThrowingAssertHandler()
    {
        try
        {
            var field = typeof(Debug).Assembly
                .GetType("System.Diagnostics.DebugProvider")
                ?.GetField("s_FailCore", BindingFlags.NonPublic | BindingFlags.Static);

            if (field?.FieldType != typeof(Action<string, string?, string?, string>))
                return;

            field.SetValue(null, new Action<string, string?, string?, string>(
                static (stackTrace, message, detailMessage, errorSource) =>
                    throw new DebugAssertException(
                        string.IsNullOrEmpty(detailMessage) ? message ?? errorSource : $"{message}\n{detailMessage}")));
        }
        catch (Exception)
        {
            // Best effort.
        }
    }
}
