using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using Unity.Robotics.UrdfImporter;

namespace RoverCompatibility.Editor
{
    /// <summary>
    /// Professional, high-performance Editor HUD for importing, auto-healing, and configuring
    /// URDF robots (Rovers, Quadrupeds, Manipulators, Bipedals) in Unity.
    /// </summary>
    public class CustomRoverImporterWindow : EditorWindow
    {
        private string _urdfPath = "";
        private string _customMeshDir = "";
        private UnityEngine.Object _urdfAsset;
        private UrdfAnalysisResult _analysis;
        private bool _showMeshList = false;
        private Vector2 _scrollPos;
        private int _selectedTab = 0;
        private readonly string[] _tabNames = new[] { "URDF Importer", "Robot Utilities", "Architecture Diagnostics" };

        private RoverMassProfile _massProfile = RoverMassProfile.AutoDetect;
        private bool _collidersConvex = true;
        private bool _stripDefaultControllers = true;
        private bool _autoConfigureRover = true;
        private bool _syncMeshesToLocalFolder = true;
        private bool _autoDetectAxis = true;
        private ImportSettings.axisType _chosenAxis = ImportSettings.axisType.yAxis;

        [MenuItem("Window/URDF Importer", false, 20)]
        [MenuItem("Tools/URDF Importer", false, 0)]
        public static void OpenWindow()
        {
            var window = GetWindow<CustomRoverImporterWindow>("URDF Importer");
            window.minSize = new Vector2(620, 680);
            window.Show();
        }

        private void OnEnable()
        {
            if (Selection.activeObject != null)
            {
                string path = AssetDatabase.GetAssetPath(Selection.activeObject);
                if (path.EndsWith(".urdf", StringComparison.OrdinalIgnoreCase))
                {
                    _urdfAsset = Selection.activeObject;
                    _urdfPath = Path.GetFullPath(path);
                    Analyze();
                }
            }
        }

        private void OnGUI()
        {
            DrawBanner();

            EditorGUILayout.Space(4);
            _selectedTab = GUILayout.Toolbar(_selectedTab, _tabNames, GUILayout.Height(30));
            EditorGUILayout.Space(6);

            _scrollPos = EditorGUILayout.BeginScrollView(_scrollPos);

            switch (_selectedTab)
            {
                case 0:
                    DrawImporterTab();
                    break;
                case 1:
                    DrawSceneUtilitiesTab();
                    break;
                case 2:
                    DrawDiagnosticsTab();
                    break;
            }

            EditorGUILayout.EndScrollView();
        }

        private void DrawBanner()
        {
            var bgStyle = new GUIStyle(GUI.skin.box)
            {
                padding = new RectOffset(16, 16, 12, 12),
                margin = new RectOffset(4, 4, 4, 4)
            };

            EditorGUILayout.BeginVertical(bgStyle);
            
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label("[URDF]", new GUIStyle(EditorStyles.label) { fontSize = 28 }, GUILayout.Width(36), GUILayout.Height(34));
            
            EditorGUILayout.BeginVertical();
            var titleStyle = new GUIStyle(EditorStyles.boldLabel)
            {
                fontSize = 15,
                normal = { textColor = EditorGUIUtility.isProSkin ? new Color(0.35f, 0.85f, 1f) : new Color(0.1f, 0.4f, 0.6f) }
            };
            EditorGUILayout.LabelField("UNIVERSAL URDF IMPORTER // ROBOTICS SUITE", titleStyle);
            EditorGUILayout.LabelField("Zero-config autonomous mesh repair, PhysX ArticulationBody motor tuning, and instant driving.", EditorStyles.miniLabel);
            EditorGUILayout.EndVertical();

            GUILayout.FlexibleSpace();
            GUILayout.Label("v1.1.0", EditorStyles.centeredGreyMiniLabel, GUILayout.Width(45));
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.EndVertical();
        }

        private void DrawImporterTab()
        {
            // 1. File Selector
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("1. Source URDF or Archive Package", EditorStyles.boldLabel);

            EditorGUI.BeginChangeCheck();
            _urdfAsset = EditorGUILayout.ObjectField("URDF / ZIP Asset", _urdfAsset, typeof(UnityEngine.Object), false);
            if (EditorGUI.EndChangeCheck() && _urdfAsset != null)
            {
                string path = AssetDatabase.GetAssetPath(_urdfAsset);
                if (path.EndsWith(".urdf", StringComparison.OrdinalIgnoreCase) || path.EndsWith(".zip", StringComparison.OrdinalIgnoreCase))
                {
                    _urdfPath = Path.GetFullPath(path);
                    Analyze();
                }
            }

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.TextField("File Path", _urdfPath);
            if (GUILayout.Button("Browse...", GUILayout.Width(80)))
            {
                string selected = EditorUtility.OpenFilePanel("Select URDF File or ZIP Archive", "Assets", "urdf,zip");
                if (!string.IsNullOrEmpty(selected))
                {
                    _urdfPath = selected;
                    string relative = "Assets" + (selected.Length > Application.dataPath.Length ? selected.Substring(Application.dataPath.Length) : "");
                    _urdfAsset = AssetDatabase.LoadAssetAtPath<UnityEngine.Object>(relative);
                    Analyze();
                }
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            _customMeshDir = EditorGUILayout.TextField("External Meshes Dir (Optional)", _customMeshDir);
            if (GUILayout.Button("Browse Dir...", GUILayout.Width(80)))
            {
                string selectedDir = EditorUtility.OpenFolderPanel("Select External Meshes Directory", "", "");
                if (!string.IsNullOrEmpty(selectedDir))
                {
                    _customMeshDir = selectedDir;
                    Analyze();
                }
            }
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();

            // 2. Pre-Import Analysis
            if (_analysis != null)
            {
                EditorGUILayout.Space(6);
                DrawAnalysisSummary();
            }

            // 3. Configuration & Optimization Settings
            EditorGUILayout.Space(6);
            DrawImportSettings();

            // 4. Action Button
            EditorGUILayout.Space(10);
            DrawActionButtons();
        }

        private void DrawAnalysisSummary()
        {
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("2. Autonomous Geometry Diagnostics", EditorStyles.boldLabel);

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField($"Total Mesh Links: {_analysis.meshItems.Count}", EditorStyles.miniBoldLabel);
            EditorGUILayout.LabelField($"Missing Paths: {_analysis.missingCount}", _analysis.missingCount > 0 ? EditorStyles.boldLabel : EditorStyles.miniLabel);
            EditorGUILayout.LabelField($"Substituted / Fallback: {_analysis.formatSubstitutedCount + _analysis.fallbackPrimitiveCount}", EditorStyles.miniLabel);
            EditorGUILayout.EndHorizontal();

            if (_analysis.missingCount > 0)
            {
                EditorGUILayout.HelpBox(
                    $"Detected {_analysis.missingCount} missing mesh file(s). Auto-healer will synthesize compatible fallback geometries and remap package:// URIs.",
                    MessageType.Warning
                );
            }
            else
            {
                EditorGUILayout.HelpBox(
                    "All mesh paths validated successfully! Ready for high-fidelity conversion.",
                    MessageType.Info
                );
            }

            _showMeshList = EditorGUILayout.Foldout(_showMeshList, $"View Link & Mesh Manifest ({_analysis.meshItems.Count})");
            if (_showMeshList)
            {
                foreach (var meshRef in _analysis.meshItems)
                {
                    EditorGUILayout.BeginHorizontal("box");
                    string icon = meshRef.status == MeshResolutionStatus.DirectlyFound ? "[Found]"
                                : meshRef.status == MeshResolutionStatus.RemappedInProject ? "[Remapped]"
                                : meshRef.status == MeshResolutionStatus.FormatSubstituted ? "[Substituted]"
                                : "[Missing]";
                    EditorGUILayout.LabelField($"{icon} {meshRef.originalUri}", EditorStyles.miniLabel);
                    EditorGUILayout.EndHorizontal();
                }
            }

            EditorGUILayout.EndVertical();
        }

        private void DrawImportSettings()
        {
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("3. Pipeline & Physics Optimization", EditorStyles.boldLabel);

            _autoDetectAxis = EditorGUILayout.Toggle(new GUIContent("Auto-Detect Coordinate Axis", "Analyzes URDF tree to select optimal vertical orientation (Y-up vs Z-up)."), _autoDetectAxis);
            if (!_autoDetectAxis)
            {
                _chosenAxis = (ImportSettings.axisType)EditorGUILayout.EnumPopup("Chosen Axis", _chosenAxis);
            }

            _autoConfigureRover = EditorGUILayout.Toggle(new GUIContent("Auto-Configure Articulation Physics", "Automatically tunes ArticulationBody drives, mass distribution, wheel spin axes, and drive controls."), _autoConfigureRover);
            _massProfile = (RoverMassProfile)EditorGUILayout.EnumPopup(new GUIContent("Mass Profile Preset", "Calibrates chassis and wheel mass profiles for optimal stability under gravity."), _massProfile);

            _collidersConvex = EditorGUILayout.Toggle(new GUIContent("Force Convex Colliders", "Ensures all link colliders are convex for PhysX ArticulationBody compliance."), _collidersConvex);
            _stripDefaultControllers = EditorGUILayout.Toggle(new GUIContent("Strip Default Controllers", "Removes default generic Unity controllers so our smart N-wheel differential drive takes over."), _stripDefaultControllers);
            _syncMeshesToLocalFolder = EditorGUILayout.Toggle(new GUIContent("Stage Meshes Locally", "Copies external CAD assets into local project folder to avoid broken package references."), _syncMeshesToLocalFolder);

            EditorGUILayout.EndVertical();
        }

        private void DrawActionButtons()
        {
            bool canImport = !string.IsNullOrEmpty(_urdfPath) && (File.Exists(_urdfPath) || _urdfPath.EndsWith(".zip", StringComparison.OrdinalIgnoreCase));
            GUI.enabled = canImport;

            var importBtnStyle = new GUIStyle(GUI.skin.button)
            {
                fontStyle = FontStyle.Bold,
                fontSize = 13,
                fixedHeight = 42
            };

            Color originalColor = GUI.backgroundColor;
            GUI.backgroundColor = canImport ? new Color(0.2f, 0.8f, 0.4f) : Color.gray;

            if (GUILayout.Button("IMPORT, HEAL & CONFIGURE ROBOT", importBtnStyle))
            {
                ExecuteFullImportPipeline();
            }

            GUI.backgroundColor = originalColor;
            GUI.enabled = true;
        }

        private void DrawSceneUtilitiesTab()
        {
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("Active Robot Quick Actions", EditorStyles.boldLabel);

            GameObject selected = Selection.activeGameObject;
            bool hasArticulation = selected != null && selected.GetComponentInChildren<ArticulationBody>() != null;

            if (hasArticulation)
            {
                var desc = RoverAnalyzer.Analyze(selected);

                // Robot Status Card
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                EditorGUILayout.LabelField($"Selected: {selected.name}", EditorStyles.boldLabel);
                EditorGUILayout.LabelField($"Architecture: {desc.RobotType}", EditorStyles.miniBoldLabel);
                EditorGUILayout.LabelField($"Mass: {desc.TotalMass:F1} kg | Articulation Bodies: {desc.AllBodies.Count} (Wheels: {desc.Wheels.Count})", EditorStyles.miniLabel);
                EditorGUILayout.EndVertical();

                EditorGUILayout.Space(6);
                EditorGUILayout.LabelField("Physics & Mesh Operations", EditorStyles.boldLabel);

                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Re-Configure Physics", GUILayout.Height(32)))
                {
                    RoverAutoConfigurator.Configure(selected, _massProfile);
                    ShowNotification(new GUIContent($"Re-configured '{selected.name}'"));
                }
                if (GUILayout.Button("Heal Meshes & Normals", GUILayout.Height(32)))
                {
                    RoverModelHealer.HealRobot(selected, out int hm, out int hmat, _urdfPath);
                    ShowNotification(new GUIContent($"Healed {hm} mesh(es), {hmat} material(s)!"));
                }
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Anti-Self-Collision", GUILayout.Height(32)))
                {
                    int pairs = RoverModelHealer.StabilizeArticulationPhysics(selected);
                    ShowNotification(new GUIContent($"Ignored {pairs} self-collision pairs"));
                }
                if (GUILayout.Button("Ground on Floor (Y=0)", GUILayout.Height(32)))
                {
                    float offset = RoverModelHealer.GroundRobotOnFloor(selected);
                    ShowNotification(new GUIContent($"Resting flush on floor (ΔY: {offset:F3}m)"));
                }
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.Space(6);
                EditorGUILayout.LabelField("Stance & Drive Presets", EditorStyles.boldLabel);

                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Apply Standing Stance", GUILayout.Height(32)))
                {
                    var controller = selected.GetComponent<ArticulatedRobotController>();
                    if (controller == null) controller = selected.AddComponent<ArticulatedRobotController>();
                    controller.InitializeJoints();
                    controller.ApplyStandStance();
                    ShowNotification(new GUIContent($"Applied standing stance"));
                }
                if (GUILayout.Button("Apply Sitting Stance", GUILayout.Height(32)))
                {
                    var controller = selected.GetComponent<ArticulatedRobotController>();
                    if (controller == null) controller = selected.AddComponent<ArticulatedRobotController>();
                    controller.InitializeJoints();
                    controller.ApplySitStance();
                    ShowNotification(new GUIContent($"Applied sitting stance"));
                }
                if (GUILayout.Button("Add Drive Controller", GUILayout.Height(32)))
                {
                    RoverAutoConfigurator.Configure(selected, _massProfile);
                    ShowNotification(new GUIContent($"Attached Differential Drive controller"));
                }
                EditorGUILayout.EndHorizontal();
            }
            else
            {
                EditorGUILayout.HelpBox(
                    "Select any robot or rover in the Hierarchy containing ArticulationBody components to use these utilities.",
                    MessageType.Info
                );
            }

            EditorGUILayout.EndVertical();
        }

        private void DrawDiagnosticsTab()
        {
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("Live Architecture & Joint Inspector", EditorStyles.boldLabel);

            GameObject selected = Selection.activeGameObject;
            if (selected == null || selected.GetComponentInChildren<ArticulationBody>() == null)
            {
                EditorGUILayout.HelpBox("Select an articulated robot in the scene to inspect its 3D spatial joint breakdown.", MessageType.None);
                EditorGUILayout.EndVertical();
                return;
            }

            var desc = RoverAnalyzer.Analyze(selected);

            EditorGUILayout.LabelField($"Robot Classification: {desc.RobotType}", EditorStyles.boldLabel);
            EditorGUILayout.LabelField($"Total Articulation Link Count: {desc.AllBodies.Count}", EditorStyles.miniLabel);

            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField($"Detected Wheels ({desc.Wheels.Count})", EditorStyles.boldLabel);
            foreach (var w in desc.Wheels)
            {
                EditorGUILayout.LabelField($"  • {w.name} (Spin Axis: {w.jointType})", EditorStyles.miniLabel);
            }

            if (desc.SteeringJoints.Count > 0)
            {
                EditorGUILayout.Space(4);
                EditorGUILayout.LabelField($"Active Steering Pivots ({desc.SteeringJoints.Count})", EditorStyles.boldLabel);
                foreach (var s in desc.SteeringJoints)
                {
                    EditorGUILayout.LabelField($"  • {s.name}", EditorStyles.miniLabel);
                }
            }

            if (desc.SuspensionJoints.Count > 0)
            {
                EditorGUILayout.Space(4);
                EditorGUILayout.LabelField($"Passive Rocker-Bogie Compliance Links ({desc.SuspensionJoints.Count})", EditorStyles.boldLabel);
                foreach (var p in desc.SuspensionJoints)
                {
                    EditorGUILayout.LabelField($"  • {p.name}", EditorStyles.miniLabel);
                }
            }

            if (desc.NonWheelJoints.Count > 0)
            {
                EditorGUILayout.Space(4);
                EditorGUILayout.LabelField($"Articulated Non-Wheel Joints ({desc.NonWheelJoints.Count})", EditorStyles.boldLabel);
                foreach (var leg in desc.NonWheelJoints)
                {
                    EditorGUILayout.LabelField($"  • {leg.name}", EditorStyles.miniLabel);
                }
            }

            EditorGUILayout.EndVertical();
        }

        private void Analyze()
        {
            if (string.IsNullOrEmpty(_urdfPath)) return;
            _analysis = UrdfMeshPathResolver.AnalyzeUrdf(_urdfPath, _customMeshDir);
        }

        public void ExecuteFullImportPipeline()
        {
            if (string.IsNullOrEmpty(_urdfPath) || !File.Exists(_urdfPath)) return;

            string importUrdfPath = _urdfPath;

            EditorUtility.DisplayProgressBar("URDF Importer", "Auto-healing URDF geometry and mesh paths...", 0.2f);
            try
            {
                importUrdfPath = UrdfMeshPathResolver.StageAndSanitizeUrdf(_urdfPath, _customMeshDir);
            }
            catch (Exception ex)
            {
                EditorUtility.ClearProgressBar();
                Debug.LogError($"[URDF Importer] Failed to stage/sanitize URDF: {ex.Message}");
                EditorUtility.DisplayDialog("Import Error", $"Failed to stage/sanitize URDF: {ex.Message}", "OK");
                return;
            }

            string urdfDirectory = Path.GetFullPath(Path.GetDirectoryName(importUrdfPath));
            UrdfAssetPathHandler.SetPackageRoot(urdfDirectory);

            UrdfRobot.collidersConvex = _collidersConvex;
            UrdfRobot.useGravity = true;
            UrdfRobot.addController = !_stripDefaultControllers;
            UrdfRobot.addFkRobot = false;

            var effectiveAxis = _autoDetectAxis
                ? UrdfMeshPathResolver.DetectOptimalAxis(importUrdfPath)
                : _chosenAxis;

            Debug.Log($"[URDF Importer] Using coordinate axis: {effectiveAxis} for '{Path.GetFileName(importUrdfPath)}'.");

            var settings = new ImportSettings
            {
                chosenAxis = effectiveAxis,
                convexMethod = ImportSettings.convexDecomposer.unity
            };

            Selection.activeObject = null;

            EditorUtility.DisplayProgressBar("URDF Importer", "Synthesizing ArticulationBody robot hierarchy...", 0.5f);
            try
            {
                var enumerator = UrdfRobotExtensions.Create(importUrdfPath, settings);
                GameObject importedRobot = null;

                bool hasNext = true;
                while (hasNext)
                {
                    try
                    {
                        hasNext = enumerator.MoveNext();
                        if (hasNext && enumerator.Current != null)
                        {
                            importedRobot = enumerator.Current;
                        }
                    }
                    catch (Exception linkEx)
                    {
                        Debug.LogWarning($"[URDF Importer] Handled notice during link construction: {linkEx.Message}");
                        break;
                    }
                }

                EditorUtility.ClearProgressBar();

                if (importedRobot == null)
                {
                    string baseName = Path.GetFileNameWithoutExtension(importUrdfPath).Replace("_sanitized", "");
                    importedRobot = GameObject.Find(baseName);
                }

                if (importedRobot == null)
                {
                    EditorUtility.DisplayDialog("Import Failed", "URDF Importer returned null. Check console for details.", "OK");
                    return;
                }

                if (_autoConfigureRover)
                {
                    RoverAutoConfigurator.Configure(importedRobot, _massProfile);
                    RoverModelHealer.HealRobot(importedRobot, out _, out _, importUrdfPath);
                    RoverModelHealer.GroundRobotOnFloor(importedRobot);
                }

                Selection.activeGameObject = importedRobot;
                if (SceneView.lastActiveSceneView != null)
                {
                    SceneView.lastActiveSceneView.FrameSelected();
                }

                Debug.Log($"<color=green>[URDF Importer] SUCCESS!</color> '{importedRobot.name}' imported and fully configured. Ready to drive!");
                ShowNotification(new GUIContent($"'{importedRobot.name}' Ready to drive!"));
            }
            catch (Exception ex)
            {
                EditorUtility.ClearProgressBar();
                Debug.LogError($"[URDF Importer] Exception during import: {ex.Message}\n{ex.StackTrace}");
                ShowNotification(new GUIContent($"Import failed: {ex.Message}"));
            }
        }
    }
}
