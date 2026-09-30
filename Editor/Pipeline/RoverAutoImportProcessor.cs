using System;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace RoverCompatibility.Editor
{
    /// <summary>
    /// Implements Option B: Fully automatic rover configuration.
    /// Monitors scene hierarchy changes, selection changes, and asset postprocessing.
    /// Whenever a new robot is imported via "Import Robot from Selected URDF file",
    /// this processor immediately detects it, validates it as a wheeled rover,
    /// and configures its physics and drive controllers automatically.
    /// </summary>
    [InitializeOnLoad]
    public class RoverAutoImportProcessor : AssetPostprocessor
    {
        private static bool _scanScheduled = false;

        static RoverAutoImportProcessor()
        {
            // Hook hierarchy and selection changes to detect freshly imported scene robots
            EditorApplication.hierarchyChanged += ScheduleScan;
            Selection.selectionChanged += OnSelectionChanged;
        }

        private static void OnSelectionChanged()
        {
            var selected = Selection.activeGameObject;
            if (selected == null) return;

            // If user selects an unconfigured robot root, schedule scan
            if (selected.GetComponent<RoverCompatibilityMarker>() == null &&
                selected.GetComponentInChildren<ArticulationBody>() != null)
            {
                ScheduleScan();
            }
        }

        private static void OnPostprocessAllAssets(
            string[] importedAssets,
            string[] deletedAssets,
            string[] movedAssets,
            string[] movedFromAssetPaths,
            bool didDomainReload)
        {
            bool relevant = importedAssets.Any(p =>
                p.EndsWith(".urdf", StringComparison.OrdinalIgnoreCase) ||
                p.EndsWith(".prefab", StringComparison.OrdinalIgnoreCase));

            if (relevant || didDomainReload)
            {
                ScheduleScan();
            }
        }

        public static void ScheduleScan()
        {
            if (_scanScheduled) return;
            _scanScheduled = true;

            EditorApplication.delayCall += () =>
            {
                _scanScheduled = false;
                ScanAndConfigureSceneRovers();
            };
        }

        public static void ScanAndConfigureSceneRovers()
        {
            if (Application.isPlaying) return;

            // Find all ArticulationBodies in the scene (including inactive)
            var allBodies = UnityEngine.Object.FindObjectsByType<ArticulationBody>(FindObjectsInactive.Include);
            if (allBodies == null || allBodies.Length == 0) return;

            var candidateRoots = allBodies
                .Where(ab => ab != null && ab.isRoot)
                .Select(ab => FindRobotRoot(ab.gameObject))
                .Where(go => go != null)
                .Distinct();

            foreach (var root in candidateRoots)
            {
                if (root == null) continue;

                // Skip if already configured
                if (root.GetComponent<RoverCompatibilityMarker>() != null)
                    continue;

                // Safety guard: ensure the robot is completely finished importing and not mid-coroutine
                var urdfRobot = root.GetComponent<Unity.Robotics.UrdfImporter.UrdfRobot>();
                if (urdfRobot != null && urdfRobot.collisionExceptions == null)
                {
                    // Official importer only assigns collisionExceptions in ImportPipelinePostCreate at the very end
                    EditorApplication.delayCall += () => ScheduleScan();
                    continue;
                }

                if (Unity.Robotics.UrdfImporter.UrdfRobotExtensions.importsettings != null &&
                    Unity.Robotics.UrdfImporter.UrdfRobotExtensions.importsettings.totalLinks > 0 &&
                    Unity.Robotics.UrdfImporter.UrdfRobotExtensions.importsettings.linksLoaded < Unity.Robotics.UrdfImporter.UrdfRobotExtensions.importsettings.totalLinks)
                {
                    // Official importer is actively in the middle of loading links
                    EditorApplication.delayCall += () => ScheduleScan();
                    continue;
                }

                var desc = RoverAnalyzer.Analyze(root);
                if (desc.IsCompatible)
                {
                    Debug.Log($"<color=cyan>[Rover & Robot Auto-Importer]</color> Detected newly imported [{desc.RobotType}]: '{root.name}' ({desc.AllBodies.Count} bodies). Configuring...");

                    // If the official importer nested the robot under whatever scene object was selected (e.g. Main Camera),
                    // immediately pull it out to the scene root so it is never hidden or deleted with the parent
                    if (root.transform.parent != null)
                    {
                        Debug.Log($"[Rover Compatibility] Moving '{root.name}' from nested parent '{root.transform.parent.name}' to Scene Root.");
                        Undo.SetTransformParent(root.transform, null, "Move Robot to Scene Root");
                    }

                    RoverAutoConfigurator.Configure(root);
                    RoverModelHealer.HealRobot(root, out _, out _);
                    RoverModelHealer.GroundRobotOnFloor(root);

                    // Select and frame the newly imported robot in the SceneView
                    Selection.activeGameObject = root;
                    if (SceneView.lastActiveSceneView != null)
                    {
                        SceneView.lastActiveSceneView.FrameSelected();
                    }
                }
            }
        }

        private static GameObject FindRobotRoot(GameObject go)
        {
            if (go == null) return null;

            // Walk upwards towards scene root or until we find a GameObject with UrdfRobot
            // or the boundary before another ArticulationBody tree
            Transform current = go.transform;
            Transform bestRoot = current;

            while (current != null)
            {
                // If it has UrdfRobot component or has RoverCompatibilityMarker
                var monoBehaviours = current.GetComponents<MonoBehaviour>();
                foreach (var mb in monoBehaviours)
                {
                    if (mb != null && (mb.GetType().Name == "UrdfRobot" || mb.GetType().Name == "RoverCompatibilityMarker"))
                    {
                        return current.gameObject;
                    }
                }

                bestRoot = current;
                // If parent has no ArticulationBody components, parent might be the container (e.g. Robot name)
                if (current.parent != null)
                {
                    // Check if parent contains UrdfRobot
                    var parentMBs = current.parent.GetComponents<MonoBehaviour>();
                    bool parentIsUrdfRobot = parentMBs.Any(m => m != null && m.GetType().Name == "UrdfRobot");
                    if (parentIsUrdfRobot)
                    {
                        return current.parent.gameObject;
                    }
                }

                current = current.parent;
            }

            return bestRoot != null ? bestRoot.gameObject : go;
        }        [MenuItem("Tools/Rover Compatibility/Configure Selected Robot", false, 20)]
        [MenuItem("Tools/URDF Importer/Configure Selected Robot", false, 20)]
        private static void ConfigureSelected()
        {
            var go = Selection.activeGameObject;
            if (go == null)
            {
                EditorUtility.DisplayDialog("Rover Compatibility", "Please select the root GameObject of your imported robot in the hierarchy first.", "OK");
                return;
            }

            // Find the root articulation or root GameObject
            var artBody = go.GetComponentInChildren<ArticulationBody>();
            if (artBody == null)
            {
                EditorUtility.DisplayDialog("Rover Compatibility", $"No ArticulationBody found on '{go.name}' or its children.", "OK");
                return;
            }

            GameObject targetRoot = go.transform.root.gameObject;
            RoverAutoConfigurator.Configure(targetRoot);
        }        [MenuItem("GameObject/Rover Compatibility/Configure as Rover", false, 10)]
        [MenuItem("GameObject/URDF Importer/Configure Selected Robot", false, 10)]
        private static void ContextMenuConfigure(MenuCommand command)
        {
            var go = command.context as GameObject;
            if (go != null)
            {
                RoverAutoConfigurator.Configure(go.transform.root.gameObject);
            }
        }
    }
}

