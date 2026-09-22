#if UNITY_EDITOR
using System;
using System.Diagnostics;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.Networking;
using Debug = UnityEngine.Debug;

/// <summary>
/// Downloads Stereolabs' simulation streamer library (the same binary the ZED Isaac Sim extension
/// ships) into Assets/Plugins/x86_64 so <c>ZedSimCamera</c> can P/Invoke it. The library is ~170 MB
/// and is git-ignored; run this once per checkout.
/// </summary>
public static class ZedSimLibraryInstaller {
    private const string Version = "5.1.1"; // CDN package version (runtime reports SDK 5.4.1)
    private const string CdnBase = "https://stereolabs.sfo2.cdn.digitaloceanspaces.com/utils/zed_isaac_sim/sl_zed/" + Version + "/";
    private const string PluginDir = "Assets/Plugins/x86_64";

    private static string Archive =>
#if UNITY_EDITOR_WIN
        $"sl_zed64_windows_x86_64_{Version}.tar.gz";
#else
        $"libsl_zed_linux_x86_64_{Version}.tar.gz";
#endif

    private static string LibraryFile =>
#if UNITY_EDITOR_WIN
        "sl_zed64.dll";
#else
        "libsl_zed.so";
#endif

    [MenuItem("Tools/ZED Sim/Install streamer library")]
    public static void Install() {
        string libraryPath = Path.Combine(PluginDir, LibraryFile);
        if (File.Exists(libraryPath) &&
            !EditorUtility.DisplayDialog("ZED Sim", $"{libraryPath} already exists. Download again?", "Re-download", "Cancel")) {
            return;
        }

        Directory.CreateDirectory(PluginDir);
        string archivePath = Path.Combine(PluginDir, Archive);
        string url = CdnBase + Archive;

        try {
            using var request = UnityWebRequest.Get(url);
            request.downloadHandler = new DownloadHandlerFile(archivePath);
            var op = request.SendWebRequest();
            while (!op.isDone) {
                if (EditorUtility.DisplayCancelableProgressBar("ZED Sim", $"Downloading {Archive}", request.downloadProgress)) {
                    request.Abort();
                    Debug.Log("ZED Sim: download cancelled.");
                    return;
                }
                System.Threading.Thread.Sleep(50);
            }
            if (request.result != UnityWebRequest.Result.Success) {
                Debug.LogError($"ZED Sim: download failed: {request.error} ({url})");
                return;
            }

            EditorUtility.DisplayProgressBar("ZED Sim", "Extracting", 1f);
            if (!RunTar(archivePath, PluginDir)) return;
        } finally {
            EditorUtility.ClearProgressBar();
            if (File.Exists(archivePath)) File.Delete(archivePath);
        }

        AssetDatabase.Refresh();
        ConfigureImporter(libraryPath);
        WarnAboutHostDependencies();
        Debug.Log($"ZED Sim: installed {libraryPath}. Enter Play mode with a ZedSimCamera in the scene to start streaming.");
    }

    private static bool RunTar(string archivePath, string destination) {
        var psi = new ProcessStartInfo("tar", $"-xzf \"{archivePath}\" -C \"{destination}\"") {
            UseShellExecute = false,
            RedirectStandardError = true,
            CreateNoWindow = true,
        };
        try {
            using var proc = Process.Start(psi);
            string err = proc.StandardError.ReadToEnd();
            proc.WaitForExit();
            if (proc.ExitCode != 0) {
                Debug.LogError($"ZED Sim: tar failed ({proc.ExitCode}): {err}");
                return false;
            }
            return true;
        } catch (Exception e) {
            Debug.LogError($"ZED Sim: could not run tar: {e.Message}");
            return false;
        }
    }

    private static void ConfigureImporter(string libraryPath) {
        var importer = AssetImporter.GetAtPath(libraryPath) as PluginImporter;
        if (importer == null) {
            Debug.LogWarning($"ZED Sim: no PluginImporter for {libraryPath}; set its platforms manually (Linux/Windows x86_64 + Editor).");
            return;
        }
        importer.SetCompatibleWithAnyPlatform(false);
        importer.SetCompatibleWithEditor(true);
#if UNITY_EDITOR_WIN
        importer.SetCompatibleWithPlatform(BuildTarget.StandaloneWindows64, true);
        importer.SetEditorData("OS", "Windows");
#else
        importer.SetCompatibleWithPlatform(BuildTarget.StandaloneLinux64, true);
        importer.SetEditorData("OS", "Linux");
#endif
        importer.SetEditorData("CPU", "x86_64");
        importer.SaveAndReimport();
    }

    private static void WarnAboutHostDependencies() {
#if UNITY_EDITOR_LINUX
        try {
            var psi = new ProcessStartInfo("ldconfig", "-p") { UseShellExecute = false, RedirectStandardOutput = true, CreateNoWindow = true };
            using var proc = Process.Start(psi);
            string output = proc.StandardOutput.ReadToEnd();
            proc.WaitForExit();
            if (!output.Contains("libturbojpeg.so.0"))
                Debug.LogWarning("ZED Sim: libturbojpeg.so.0 is not installed; libsl_zed will fail to load. Run `sudo apt install libturbojpeg`.");
            if (!output.Contains("libnvidia-encode.so.1"))
                Debug.LogWarning("ZED Sim: libnvidia-encode.so.1 not found; the streamer needs an NVIDIA driver with NVENC.");
        } catch (Exception) {
            // ldconfig missing: nothing to check.
        }
#endif
    }
}
#endif
