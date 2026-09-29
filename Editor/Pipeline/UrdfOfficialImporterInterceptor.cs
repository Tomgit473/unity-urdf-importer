using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using Unity.Robotics.UrdfImporter.Editor;

namespace RoverCompatibility.Editor
{
    /// <summary>
    /// Intercepts and monitors the official Unity URDF Importer to prevent the two primary traps:
    /// 1. Parenting Trap: The official importer parents the imported robot under whatever GameObject
    ///    happens to be selected in the Hierarchy (e.g. Main Camera), hiding it from the user.
    ///    This interceptor ensures scene selection is cleared whenever a URDF import begins.
    /// 2. Missing Mesh / Coroutine Crash Trap: Auto-heals relative '../' mesh paths, formats, and bakes
    ///    prefabs before the official importer runs, preventing "Could not find file" exceptions.
    /// 3. Top-Priority Menu: Places "🚀 Import Drivable Rover" at priority -100 so it appears at the
    ///    very top of the Project context menu.
    /// </summary>
    [InitializeOnLoad]
    public static class UrdfOfficialImporterInterceptor
    {
        private static string _lastProcessedUrdf = "";

        static UrdfOfficialImporterInterceptor()
        {
            EditorApplication.update += MonitorOfficialImportMenu;
        }

        [MenuItem("Assets/Import Robot (URDF)", true, 18)]
        public static bool ValidateTopLevelImportRover()
        {
            if (Selection.activeObject == null) return false;
            string path = AssetDatabase.GetAssetPath(Selection.activeObject);
            if (string.IsNullOrEmpty(path)) return false;
            if (path.EndsWith(".urdf", StringComparison.OrdinalIgnoreCase) || path.EndsWith(".zip", StringComparison.OrdinalIgnoreCase)) return true;
            if (Directory.Exists(path))
            {
                return Directory.GetFiles(path, "*.urdf", SearchOption.AllDirectories).Length > 0 ||
                       Directory.GetFiles(path, "*.zip", SearchOption.AllDirectories).Length > 0;
            }
            return false;
        }

        [MenuItem("Assets/Import Robot (URDF)", false, 18)]
        public static void TopLevelImportRover()
        {
            if (Selection.activeObject == null) return;
            string assetPath = AssetDatabase.GetAssetPath(Selection.activeObject);
            if (!assetPath.EndsWith(".urdf", StringComparison.OrdinalIgnoreCase) && !assetPath.EndsWith(".zip", StringComparison.OrdinalIgnoreCase)) return;

            // Clear active scene selection so the robot is NEVER parented under a scene object
            Selection.activeObject = null;
            RoverContextMenuImporter.ImportRoverAtPath(assetPath);
        }

        [MenuItem("Assets/Open URDF Importer Window", false, 19)]
        public static void OpenImporterWindowFromAssets()
        {
            CustomRoverImporterWindow.OpenWindow();
        }


        private static void MonitorOfficialImportMenu()
        {
            if (Application.isPlaying) return;

            var importMenus = Resources.FindObjectsOfTypeAll<FileImportMenu>();
            if (importMenus == null || importMenus.Length == 0)
            {
                _lastProcessedUrdf = "";
                return;
            }

            foreach (var window in importMenus)
            {
                if (window == null) continue;

                // TRAP 1 PREVENTION: If a scene GameObject is currently selected, clear it
                // so the official importer's GameObjectUtility.SetParentAndAlign will place the robot at scene root!
                if (Selection.activeGameObject != null)
                {
                    Selection.activeObject = null;
                }

                // TRAP 2 PREVENTION: If the window is pointing to a raw URDF that needs path/mesh healing
                if (!string.IsNullOrEmpty(window.urdfFile) && File.Exists(window.urdfFile))
                {
                    if (_lastProcessedUrdf != window.urdfFile && !window.urdfFile.EndsWith("_sanitized.urdf", StringComparison.OrdinalIgnoreCase))
                    {
                        _lastProcessedUrdf = window.urdfFile;
                        try
                        {
                            var analysis = UrdfMeshPathResolver.AnalyzeUrdf(window.urdfFile);
                            bool needsSanitization = analysis.remappedCount > 0 ||
                                                     analysis.formatSubstitutedCount > 0 ||
                                                     analysis.missingWheelColliderCount > 0 ||
                                                     analysis.zeroMassLinksCount > 0 ||
                                                     analysis.meshItems.Any(m => m.originalUri.StartsWith("package://") ||
                                                                                  m.originalUri.StartsWith("../") ||
                                                                                  m.originalUri.StartsWith("..\\") ||
                                                                                  m.originalUri.EndsWith(".gltf", StringComparison.OrdinalIgnoreCase));

                            if (needsSanitization)
                            {
                                Debug.Log($"[Rover Compatibility] Auto-healing URDF mesh paths for official importer: {Path.GetFileName(window.urdfFile)}");
                                string sanitizedPath = UrdfMeshPathResolver.StageAndSanitizeUrdf(window.urdfFile);
                                window.urdfFile = sanitizedPath;
                                window.Repaint();
                            }
                        }
                        catch (Exception ex)
                        {
                            Debug.LogWarning($"[Rover Compatibility] Note: Interceptor check completed with notice: {ex.Message}");
                        }
                    }
                }
            }
        }
    }
}
