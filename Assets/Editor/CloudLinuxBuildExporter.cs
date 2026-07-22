using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

public static class CloudLinuxBuildExporter
{
    private const string ScenePath = "Assets/Scenes/SampleScene.unity";
    private const string OutputDirectory = "CloudBuild_Linux";
    private const string ExecutablePath = OutputDirectory + "/GFSX_Simulator.x86_64";

    [MenuItem("Tools/URFU/Build Linux Cloud Training")]
    public static void Build()
    {
        if (Directory.Exists(OutputDirectory))
        {
            Directory.Delete(OutputDirectory, true);
        }

        Directory.CreateDirectory(OutputDirectory);
        var options = new BuildPlayerOptions
        {
            scenes = new[] { ScenePath },
            locationPathName = ExecutablePath,
            target = BuildTarget.StandaloneLinux64,
            // Regular Linux Player (not the Dedicated Server subtarget): headless
            // training uses --no-graphics anyway, and this builds with the standard
            // "Linux Build Support" module instead of requiring the separate
            // "Linux Dedicated Server Build Support" one.
            subtarget = (int)StandaloneBuildSubtarget.Player,
            options = BuildOptions.CleanBuildCache
        };

        BuildReport report = BuildPipeline.BuildPlayer(options);
        if (report.summary.result != BuildResult.Succeeded)
        {
            throw new InvalidOperationException(
                $"Linux build failed: {report.summary.result}, errors: {report.summary.totalErrors}");
        }

        File.Copy("config.yaml", Path.Combine(OutputDirectory, "config.yaml"), true);
        string pluginsDirectory = Path.Combine(OutputDirectory, "GFSX_Simulator_Data", "Plugins");
        string grpcSource = Path.Combine(pluginsDirectory, "AnyCPU", "libgrpc_csharp_ext.x64.so");
        string nativePluginDirectory = Path.Combine(pluginsDirectory, "x86_64");
        if (File.Exists(grpcSource))
        {
            Directory.CreateDirectory(nativePluginDirectory);
            File.Copy(grpcSource,
                Path.Combine(nativePluginDirectory, "libgrpc_csharp_ext.x64.so"), true);
        }

        File.WriteAllText(Path.Combine(OutputDirectory, "START_TRAINING.txt"),
            "sudo apt-get update && sudo apt-get install -y libgtk-3-0 unzip\n" +
            "bash PREPARE_LINUX.sh\n" +
            "mlagents-learn config.yaml --run-id=cloud_reward_fixed " +
            "--env=./GFSX_Simulator.x86_64 --no-graphics\n");
        File.WriteAllText(Path.Combine(OutputDirectory, "PREPARE_LINUX.sh"),
            "#!/usr/bin/env bash\n" +
            "set -e\n" +
            "ROOT=\"$(cd \"$(dirname \"$0\")\" && pwd)\"\n" +
            "chmod 755 \"$ROOT/GFSX_Simulator.x86_64\" \"$ROOT/UnityPlayer.so\"\n" +
            "chmod 755 \"$ROOT/GFSX_Simulator_Data\"\n" +
            "chmod -R a+rX \"$ROOT/GFSX_Simulator_Data\"\n" +
            "find \"$ROOT/GFSX_Simulator_Data\" -type d -exec chmod 755 {} +\n" +
            "find \"$ROOT/GFSX_Simulator_Data\" -type f -name '*.so' -exec chmod 755 {} +\n" +
            "find \"$ROOT\" -maxdepth 1 -type f -name '*.so*' -exec chmod 755 {} +\n" +
            "echo 'Linux permissions prepared successfully.'\n");

        Debug.Log($"CLOUD_LINUX_BUILD PASSED|path={Path.GetFullPath(OutputDirectory)}|" +
                  $"bytes={report.summary.totalSize}");
    }
}
