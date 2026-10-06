using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;

namespace Robust.Client.Audio;

internal sealed partial class AudioManager
{
    private static int _openAlPreloaded;

    /// <summary>
    /// Preloads the bundled OpenAL (OpenAL Soft) before OpenTK loads it by name, so the process does not pick up a
    /// system implementation instead. On Windows an old Creative OpenAL32.dll in System32 wins the DLL search over
    /// the copy in runtimes/, and it has no EFX: occlusion low-pass and reverb silently do nothing and every effect
    /// call fails with AL errors. Once the library is loaded, later loads by name return the same module.
    /// </summary>
    private void PreloadOpenAl()
    {
        if (Interlocked.Exchange(ref _openAlPreloaded, 1) != 0)
            return;

        (string Rid, string FileName) platform;
        var arm64 = RuntimeInformation.ProcessArchitecture == Architecture.Arm64;

        if (OperatingSystem.IsWindows())
        {
            platform = arm64 ? ("win-arm64", "OpenAL32.dll")
                : Environment.Is64BitProcess ? ("win-x64", "OpenAL32.dll")
                : ("win-x86", "OpenAL32.dll");
        }
        else if (OperatingSystem.IsLinux())
        {
            platform = arm64 ? ("linux-arm64", "libopenal.so.1") : ("linux-x64", "libopenal.so.1");
        }
        else if (OperatingSystem.IsMacOS())
        {
            platform = arm64 ? ("osx-arm64", "libopenal.1.dylib") : ("osx-x64", "libopenal.1.dylib");
        }
        else
        {
            OpenALSawmill.Info("No bundled OpenAL for this platform, using the system implementation.");
            return;
        }

        var baseDir = AppContext.BaseDirectory;
        string[] candidates =
        [
            Path.Combine(baseDir, "runtimes", platform.Rid, "native", platform.FileName),
            Path.Combine(baseDir, platform.FileName),
        ];

        foreach (var candidate in candidates)
        {
            if (!File.Exists(candidate))
            {
                OpenALSawmill.Debug("No bundled OpenAL at {0}, trying next candidate.", candidate);
                continue;
            }

            try
            {
                NativeLibrary.Load(candidate);
                OpenALSawmill.Info("Preloaded bundled OpenAL from {0}", candidate);
                return;
            }
            catch (Exception ex)
            {
                OpenALSawmill.Error("Found bundled OpenAL at {0} but failed to load it. {1}", candidate, ex);
            }
        }

        OpenALSawmill.Warning("No usable bundled OpenAL found for {0}, falling back to the system implementation.", platform.Rid);
    }
}
