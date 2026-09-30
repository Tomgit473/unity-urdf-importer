using System;
using System.IO;
using UnityEditor;
using UnityEngine;
using Unity.Robotics.UrdfImporter;

namespace RoverCompatibility.Editor
{
    public static class RoverContextMenuImporter
    {        [MenuItem("Assets/Rover Compatibility/Fix & Import Drivable Rover", true, 20)]
        [MenuItem("Assets/URDF Importer/Import and Configure Robot", true, 20)]
        public static bool ValidateImportRover()
        {
            if (Selection.activeObject == null) return false;
            string path = AssetDatabase.GetAssetPath(Selection.activeObject);
            if (path.EndsWith(".urdf", StringComparison.OrdinalIgnoreCase) || path.EndsWith(".zip", StringComparison.OrdinalIgnoreCase)) return true;
            if (Directory.Exists(path))
            {
                return Directory.GetFiles(path, "*.urdf", SearchOption.AllDirectories).Length > 0 ||
                       Directory.GetFiles(path, "*.zip", SearchOption.AllDirectories).Length > 0;
            }
            return false;
        }        [MenuItem("Assets/Rover Compatibility/Fix & Import Drivable Rover", false, 20)]
        [MenuItem("Assets/URDF Importer/Import and Configure Robot", false, 20)]
        public static void ImportSelectedRover()
        {
            if (Selection.activeObject == null) return;
            string assetPath = AssetDatabase.GetAssetPath(Selection.activeObject);
            ImportRoverAtPath(assetPath);
        }

        public static GameObject ImportRoverAtPath(string inputPath, RoverMassProfile massProfile = RoverMassProfile.AutoDetect, string customMeshDir = null)
        {
            string fullUrdfPath = Path.GetFullPath(inputPath);
            Debug.Log($"[Rover Importer] Starting One-Click Auto-Healing & Import for: {fullUrdfPath}");

            // 1. Stage external URDFs and auto-heal meshes/physics
            EditorUtility.DisplayProgressBar("Rover Compatibility", "Auto-healing URDF meshes and physics...", 0.2f);
            string sanitizedUrdfPath;
            try
            {
                sanitizedUrdfPath = UrdfMeshPathResolver.StageAndSanitizeUrdf(fullUrdfPath, customMeshDir);
            }
            catch (Exception ex)
            {
                EditorUtility.ClearProgressBar();
                Debug.LogError($"[Rover Importer] Failed to stage/sanitize URDF: {ex.Message}");
                return null;
            }

            // 2. Set package root
            string urdfDirectory = Path.GetFullPath(Path.GetDirectoryName(sanitizedUrdfPath));
            UrdfAssetPathHandler.SetPackageRoot(urdfDirectory);

            // 3. Configure import settings
            UrdfRobot.collidersConvex = true;
            UrdfRobot.useGravity = true;
            UrdfRobot.addController = false; // Strip default position controllers
            UrdfRobot.addFkRobot = false;

            var optimalAxis = UrdfMeshPathResolver.DetectOptimalAxis(sanitizedUrdfPath);
            Debug.Log($"[Rover Importer] Coordinate axis configured to: {optimalAxis} for '{Path.GetFileName(sanitizedUrdfPath)}'.");

            var settings = new ImportSettings
            {
                chosenAxis = optimalAxis,
                convexMethod = ImportSettings.convexDecomposer.unity
            };

            // Clear active selection to ensure the newly created rover is at scene root and never parented under an existing object
            Selection.activeObject = null;

            // 4. Run import with safe coroutine stepping
            EditorUtility.DisplayProgressBar("Rover Compatibility", "Importing URDF hierarchy...", 0.6f);
            GameObject importedRover = null;
            try
            {
                var enumerator = UrdfRobotExtensions.Create(sanitizedUrdfPath, settings);

                bool hasNext = true;
                while (hasNext)
                {
                    try
                    {
                        hasNext = enumerator.MoveNext();
                        if (hasNext && enumerator.Current != null)
                        {
                            importedRover = enumerator.Current;
                        }
                    }
                    catch (Exception linkEx)
                    {
                        Debug.LogWarning($"[Rover Importer] Handled notice during link construction: {linkEx.Message}");
                        break;
                    }
                }

                EditorUtility.ClearProgressBar();

                // Fallback root search if enumerator did not yield the root reference directly
                if (importedRover == null)
                {
                    try
                    {
                        var doc = System.Xml.Linq.XDocument.Load(sanitizedUrdfPath);
                        string robotName = doc.Root?.Attribute("name")?.Value;
                        if (!string.IsNullOrEmpty(robotName))
                        {
                            importedRover = GameObject.Find(robotName);
                        }
                    }
                    catch { }

                    if (importedRover == null)
                    {
                        var allRobots = UnityEngine.Object.FindObjectsByType<UrdfRobot>(FindObjectsInactive.Include);
                        if (allRobots != null && allRobots.Length > 0)
                        {
                            importedRover = allRobots[allRobots.Length - 1].gameObject;
                        }
                    }

                    if (importedRover == null)
                    {
                        string baseName = Path.GetFileNameWithoutExtension(sanitizedUrdfPath).Replace("_sanitized", "");
                        importedRover = GameObject.Find(baseName);
                    }
                }

                if (importedRover == null)
                {
                    Debug.LogError("[Rover Importer] URDF Importer returned null. Check console for details.");
                    return null;
                }

                // 5. Post-Import auto configuration & Mesh/Material healing
                RoverAutoConfigurator.Configure(importedRover, massProfile);
                RoverModelHealer.HealRobot(importedRover, out int healedM, out int healedMat, sanitizedUrdfPath);
                RoverModelHealer.GroundRobotOnFloor(importedRover);

                // 6. Select and focus
                Selection.activeGameObject = importedRover;
                if (SceneView.lastActiveSceneView != null)
                {
                    SceneView.lastActiveSceneView.FrameSelected();
                    SceneView.lastActiveSceneView.ShowNotification(new GUIContent($"'{importedRover.name}' Ready to drive!"));
                }

                Debug.Log($"<color=green>[Rover Importer] SUCCESS!</color> '{importedRover.name}' imported and fully configured. Ready to drive!");
                return importedRover;
            }
            catch (Exception ex)
            {
                EditorUtility.ClearProgressBar();
                Debug.LogError($"[Rover Importer] Exception during import: {ex.Message}\n{ex.StackTrace}");
                return null;
            }
        }
    }
}
