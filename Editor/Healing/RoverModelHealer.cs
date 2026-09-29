using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using UnityEditor;
using UnityEngine;
using Unity.Robotics.UrdfImporter;

namespace RoverCompatibility.Editor
{
    internal class UrdfMaterialData
    {
        public string name;
        public Color? color;
        public string texturePath;
    }

    internal class UrdfVisualData
    {
        public string linkName;
        public string visualName;
        public string geometryType;
        public string meshFile;
        public Vector3? scale;
        public string materialName;
        public Color? inlineColor;
        public string inlineTexture;
    }

    internal class UrdfLinkData
    {
        public string name;
        public string parentJoint;
        public string parentLink;
        public string jointType;
    }

    [AddComponentMenu("")]
    internal class UrdfPrimitiveVisualAxisFixed : MonoBehaviour { }

    /// <summary>
    /// Master Mesh, Material, and Hierarchy Healing Engine for URDF Robots in Unity.
    /// Solves all post-import visual, material, and structural defects:
    /// 1. Missing / NULL Meshes: Re-associates unlinked MeshFilters and un-instantiated UrdfVisuals
    ///    with their source .asset, .stl, .dae, .obj, .fbx, or .prefab files, or injects procedural fallbacks.
    /// 2. Missing / Pink / Incompatible Materials: Upgrades materials to active pipeline (Universal Render Pipeline /
    ///    URP Lit, HDRP, or Standard), supporting multi-submesh models without stripping, and applying exact
    ///    URDF RGBA colors and textures.
    /// 3. Incomplete Robot / Missing Links: Compares hierarchy against URDF XML and constructs any dropped
    ///    or un-instantiated links with their PhysX ArticulationBody, visuals, and colliders.
    /// 4. Convex Colliders: Ensures all MeshColliders have valid sharedMesh and convex = true for PhysX stability.
    /// </summary>
    public static class RoverModelHealer
    {
        private static readonly Dictionary<string, Material> _healedMaterialCache = new Dictionary<string, Material>(StringComparer.OrdinalIgnoreCase);
        private static readonly Dictionary<string, Mesh> _healedMeshCache = new Dictionary<string, Mesh>(StringComparer.OrdinalIgnoreCase);
        private static readonly Dictionary<string, Texture> _healedTextureCache = new Dictionary<string, Texture>(StringComparer.OrdinalIgnoreCase);

        [MenuItem("GameObject/Rover Compatibility/Heal Meshes & Materials", false, 10)]
        public static void HealSelectedGameObject()
        {
            if (Selection.activeGameObject == null)
            {
                EditorUtility.DisplayDialog("Heal Rover", "Please select a Rover GameObject in the Hierarchy.", "OK");
                return;
            }

            int healedMeshes, healedMats;
            HealRobot(Selection.activeGameObject, out healedMeshes, out healedMats);
            EditorUtility.DisplayDialog("Rover Healer Complete",
                $"Successfully healed '{Selection.activeGameObject.name}':\n" +
                $"• Restored / Verified Meshes: {healedMeshes}\n" +
                $"• Upgraded Materials: {healedMats}", "OK");
        }

        [MenuItem("GameObject/Rover Compatibility/🛡️ Stabilize Articulation & Eliminate Self-Collisions", false, 11)]
        public static void StabilizeSelectedArticulation()
        {
            if (Selection.activeGameObject == null)
            {
                EditorUtility.DisplayDialog("Stabilize Robot", "Please select a Robot/Rover GameObject in the Hierarchy.", "OK");
                return;
            }

            int count = StabilizeArticulationPhysics(Selection.activeGameObject);
            EditorUtility.DisplayDialog("Robot Physics Stabilized",
                $"Successfully stabilized '{Selection.activeGameObject.name}':\n" +
                $"• Ignored {count} intra-robot collision pair(s) to prevent joint explosions.\n" +
                $"• Upgraded ArticulationBody solver iterations to 24.\n" +
                $"• Balanced micro-masses, inertia tensors, and limb holding drives.", "OK");
        }

        public static void HealRobot(GameObject robotRoot)
        {
            HealRobot(robotRoot, out _, out _);
        }

        public static void HealRobot(GameObject robotRoot, out int healedMeshes, out int healedMaterials, string urdfPathOrDirectory = null)
        {
            healedMeshes = 0;
            healedMaterials = 0;

            if (robotRoot == null) return;

            _healedMaterialCache.Clear();
            _healedMeshCache.Clear();
            _healedTextureCache.Clear();

            // 1. Determine URDF file and meshes folder
            string urdfFilePath = null;
            string localMeshesDir = null;

            if (!string.IsNullOrEmpty(urdfPathOrDirectory))
            {
                if (File.Exists(urdfPathOrDirectory))
                {
                    urdfFilePath = urdfPathOrDirectory;
                    localMeshesDir = Path.GetDirectoryName(urdfPathOrDirectory);
                }
                else if (Directory.Exists(urdfPathOrDirectory))
                {
                    localMeshesDir = urdfPathOrDirectory;
                    var foundUrdfs = Directory.GetFiles(urdfPathOrDirectory, "*.urdf", SearchOption.AllDirectories);
                    if (foundUrdfs.Length > 0)
                    {
                        urdfFilePath = foundUrdfs.FirstOrDefault(f => f.EndsWith("_sanitized.urdf", StringComparison.OrdinalIgnoreCase)) ?? foundUrdfs[0];
                    }
                }
            }

            if (string.IsNullOrEmpty(localMeshesDir))
            {
                localMeshesDir = FindMeshesDirectoryForRobot(robotRoot.name);
            }

            if (string.IsNullOrEmpty(urdfFilePath))
            {
                urdfFilePath = FindUrdfFileForRobot(robotRoot.name);
            }

            // 2. Parse URDF XML metadata for true colors, textures, and links
            Dictionary<string, UrdfMaterialData> globalMaterials;
            Dictionary<string, List<UrdfVisualData>> linkVisuals;
            List<UrdfLinkData> allUrdfLinks;
            ParseUrdfData(urdfFilePath, out globalMaterials, out linkVisuals, out allUrdfLinks);

            // 3. Detect active Render Pipeline shader
            var currentRP = UnityEngine.Rendering.GraphicsSettings.currentRenderPipeline ?? 
                            UnityEngine.Rendering.GraphicsSettings.defaultRenderPipeline ?? 
                            UnityEngine.QualitySettings.renderPipeline;
            bool isURP = currentRP != null && currentRP.GetType().Name.Contains("Universal");
            bool isHDRP = currentRP != null && currentRP.GetType().Name.Contains("HighDefinition");

            Shader targetShader = null;
            if (isURP)
            {
                targetShader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("URP/Lit") ?? Shader.Find("Universal Render Pipeline/Simple Lit");
            }
            else if (isHDRP)
            {
                targetShader = Shader.Find("HDRP/Lit");
            }

            if (targetShader == null)
            {
                targetShader = Shader.Find("Standard") ?? Shader.Find("Mobile/Diffuse") ?? Shader.Find("Diffuse");
            }

            // 4. Phase 1: Reconstruct any missing links from the URDF that the official importer dropped
            if (allUrdfLinks.Count > 0)
            {
                var existingLinks = new HashSet<string>(
                    robotRoot.GetComponentsInChildren<UrdfLink>(true).Select(l => l.name),
                    StringComparer.OrdinalIgnoreCase
                );

                foreach (var uld in allUrdfLinks)
                {
                    if (!existingLinks.Contains(uld.name))
                    {
                        Transform parentTransform = robotRoot.transform;
                        if (!string.IsNullOrEmpty(uld.parentLink))
                        {
                            var pLink = robotRoot.GetComponentsInChildren<UrdfLink>(true)
                                .FirstOrDefault(l => string.Equals(l.name, uld.parentLink, StringComparison.OrdinalIgnoreCase));
                            if (pLink != null) parentTransform = pLink.transform;
                        }

                        GameObject newLinkGo = new GameObject(uld.name);
                        newLinkGo.transform.SetParent(parentTransform, false);
                        newLinkGo.AddComponent<UrdfLink>();

                        var ab = newLinkGo.AddComponent<ArticulationBody>();
                        if (uld.jointType == "revolute" || uld.jointType == "continuous")
                        {
                            ab.jointType = ArticulationJointType.RevoluteJoint;
                        }
                        else if (uld.jointType == "prismatic")
                        {
                            ab.jointType = ArticulationJointType.PrismaticJoint;
                        }
                        else
                        {
                            ab.jointType = ArticulationJointType.FixedJoint;
                        }

                        // Create Visuals container & child ONLY if URDF actually defines visuals for this link
                        if (linkVisuals.TryGetValue(uld.name, out var uldVisuals) && uldVisuals.Count > 0)
                        {
                            GameObject visualsGroup = new GameObject("Visuals");
                            visualsGroup.transform.SetParent(newLinkGo.transform, false);
                            visualsGroup.AddComponent<UrdfVisuals>();

                            GameObject visualChild = new GameObject("unnamed");
                            visualChild.transform.SetParent(visualsGroup.transform, false);
                            visualChild.AddComponent<UrdfVisual>();

                            Mesh mesh = ResolveMeshAsset(uld.name, uld.name, localMeshesDir);
                            if (mesh != null)
                            {
                                var mf = visualChild.AddComponent<MeshFilter>();
                                mf.sharedMesh = mesh;
                                var mr = visualChild.AddComponent<MeshRenderer>();
                                mr.sharedMaterial = CreateHealedMaterial(uld.name, null, targetShader, isURP, globalMaterials, linkVisuals, localMeshesDir);
                                healedMeshes++;
                            }
                        }

                        // Create Collisions container & child
                        GameObject colGroup = new GameObject("Collisions");
                        colGroup.transform.SetParent(newLinkGo.transform, false);
                        colGroup.AddComponent<UrdfCollisions>();

                        GameObject colChild = new GameObject("Collision");
                        colChild.transform.SetParent(colGroup.transform, false);
                        var mc = colChild.AddComponent<MeshCollider>();
                        mc.convex = true;

                        existingLinks.Add(uld.name);
                        Debug.Log($"<color=green>[Rover Model Healer]</color> Restored missing robot link: '{uld.name}' under '{parentTransform.name}'.");
                    }
                }
            }

            // Cleanup any duplicate procedural fallback bumper spheres if real meshes are present
            var allFallbacks = robotRoot.GetComponentsInChildren<Transform>(true)
                .Where(t => t != null && t.name.EndsWith("_Mesh") && t.GetComponent<MeshFilter>()?.sharedMesh?.name.Contains("sphere_fallback") == true)
                .ToList();

            foreach (var fb in allFallbacks)
            {
                if (fb == null) continue;
                UrdfLink pLink = fb.GetComponentInParent<UrdfLink>();
                if (pLink != null)
                {
                    bool hasRealMesh = pLink.GetComponentsInChildren<MeshFilter>(true)
                        .Any(mf => mf != null && mf.gameObject != fb.gameObject && mf.sharedMesh != null && !mf.sharedMesh.name.Contains("sphere_fallback"));
                    if (hasRealMesh)
                    {
                        UnityEngine.Object.DestroyImmediate(fb.gameObject);
                        Debug.Log($"[Rover Model Healer] Removed redundant fallback bumper '{fb.name}' because authentic mesh part is present.");
                    }
                }
            }

            // 4.5 Phase 1.5: Fix primitive visual rotations for robots imported with Z-Axis Up
            // In Unity's official UrdfRobotExtensions.CorrectAxis, visual.transform.localRotation is multiplied
            // by Euler(-90, 0, 90) for ALL UrdfVisual components without checking if geometryType == Mesh.
            // For primitive Box, Cylinder, and Sphere visuals, this erroneously twists the geometry 90 degrees away
            // from its link and its Box/Cylinder collider. We reverse that erroneous rotation here.
            var robotUrdf = robotRoot.GetComponent<UrdfRobot>();
            if (robotUrdf != null && robotUrdf.chosenAxis == ImportSettings.axisType.zAxis)
            {
                Quaternion correction = Quaternion.Euler(-90, 0, 90);
                Quaternion invCorrection = Quaternion.Inverse(correction);
                var allVisualComponents = robotRoot.GetComponentsInChildren<UrdfVisual>(true);
                foreach (var uv in allVisualComponents)
                {
                    if (uv != null && uv.geometryType != GeometryTypes.Mesh)
                    {
                        if (uv.GetComponent<UrdfPrimitiveVisualAxisFixed>() == null)
                        {
                            uv.transform.localRotation = uv.transform.localRotation * invCorrection;
                            uv.gameObject.AddComponent<UrdfPrimitiveVisualAxisFixed>();
                        }
                    }
                }
            }

            // 5. Phase 2: Heal empty UrdfVisual components that failed to instantiate meshes
            var allVisuals = robotRoot.GetComponentsInChildren<UrdfVisual>(true);
            foreach (var uv in allVisuals)
            {
                UrdfLink uLink = uv.GetComponentInParent<UrdfLink>();
                string linkName = uLink != null ? uLink.name : uv.name;

                // Check if this visual or parent link already has valid authentic mesh filters
                var existingFilters = (uLink != null ? uLink.GetComponentsInChildren<MeshFilter>(true) : uv.GetComponentsInChildren<MeshFilter>(true))
                    .Where(f => f != null && f.sharedMesh != null && !f.name.EndsWith("_Mesh"))
                    .ToList();

                if (existingFilters.Count > 0)
                {
                    // The link already has its authentic parts - NEVER create a duplicate fallback bumper!
                    continue;
                }

                var mf = uv.GetComponentInChildren<MeshFilter>(true);
                if (mf == null || mf.sharedMesh == null)
                {
                    Mesh recovered = null;
                    if (linkVisuals.TryGetValue(linkName, out var vList) && vList.Count > 0)
                    {
                        var vItem = vList.FirstOrDefault(x => !string.IsNullOrEmpty(x.meshFile));
                        if (vItem != null)
                        {
                            recovered = ResolveMeshAsset(Path.GetFileName(vItem.meshFile), linkName, localMeshesDir);
                        }
                        else
                        {
                            var primVisual = vList.FirstOrDefault(x => x.geometryType == "box" || x.geometryType == "cylinder" || x.geometryType == "sphere");
                            if (primVisual != null)
                            {
                                GeometryTypes gType = primVisual.geometryType == "cylinder" ? GeometryTypes.Cylinder :
                                                      (primVisual.geometryType == "sphere" ? GeometryTypes.Sphere : GeometryTypes.Box);
                                recovered = CreateProceduralFallbackForLink(linkName, gType);
                            }
                        }
                    }

                    if (recovered == null)
                    {
                        recovered = ResolveMeshAsset(uv.name, linkName, localMeshesDir);
                    }

                    if (recovered == null && (uv.geometryType == GeometryTypes.Cylinder || uv.geometryType == GeometryTypes.Box))
                    {
                        recovered = CreateProceduralFallbackForLink(linkName, uv.geometryType);
                    }

                    if (recovered != null)
                    {
                        GameObject meshChild = new GameObject($"{linkName}_Mesh");
                        meshChild.transform.SetParent(uv.transform, false);
                        var newMf = meshChild.AddComponent<MeshFilter>();
                        newMf.sharedMesh = recovered;
                        var newMr = meshChild.AddComponent<MeshRenderer>();
                        newMr.sharedMaterial = CreateHealedMaterial(linkName, null, targetShader, isURP, globalMaterials, linkVisuals, localMeshesDir);
                        healedMeshes++;
                    }
                }
            }

            // 6. Phase 3: Mesh Recovery for all existing MeshFilters
            var filters = robotRoot.GetComponentsInChildren<MeshFilter>(true);
            foreach (var mf in filters)
            {
                if (mf.sharedMesh == null)
                {
                    string objectName = mf.gameObject.name;
                    var linkTransform = mf.GetComponentInParent<Unity.Robotics.UrdfImporter.UrdfLink>()?.transform;
                    string linkName = linkTransform != null ? linkTransform.name : (mf.transform.parent != null ? mf.transform.parent.name : "");

                    Mesh recovered = null;
                    if (!string.IsNullOrEmpty(linkName) && linkVisuals.TryGetValue(linkName, out var vList) && vList.Count > 0)
                    {
                        var vItem = vList.FirstOrDefault(x => !string.IsNullOrEmpty(x.meshFile));
                        if (vItem != null)
                        {
                            recovered = ResolveMeshAsset(Path.GetFileName(vItem.meshFile), linkName, localMeshesDir);
                        }
                        else
                        {
                            var primVisual = vList.FirstOrDefault(x => x.geometryType == "box" || x.geometryType == "cylinder" || x.geometryType == "sphere");
                            if (primVisual != null)
                            {
                                GeometryTypes gType = primVisual.geometryType == "cylinder" ? GeometryTypes.Cylinder :
                                                      (primVisual.geometryType == "sphere" ? GeometryTypes.Sphere : GeometryTypes.Box);
                                recovered = CreateProceduralFallbackForLink(linkName, gType);
                            }
                        }
                    }

                    if (recovered == null)
                    {
                        recovered = ResolveMeshAsset(objectName, linkName, localMeshesDir);
                    }

                    if (recovered != null)
                    {
                        mf.sharedMesh = recovered;
                        healedMeshes++;
                    }
                    else
                    {
                        // Check if the link already possesses an authentic CAD mesh filter before generating a fallback
                        bool linkAlreadyHasMesh = false;
                        if (linkTransform != null)
                        {
                            linkAlreadyHasMesh = linkTransform.GetComponentsInChildren<MeshFilter>(true)
                                .Any(f => f != mf && f.sharedMesh != null && !f.sharedMesh.name.Contains("fallback"));
                        }

                        if (!linkAlreadyHasMesh)
                        {
                            Mesh fallbackMesh = CreateProceduralFallbackForLink(linkName, GeometryTypes.Mesh);
                            mf.sharedMesh = fallbackMesh;
                            healedMeshes++;
                        }
                    }
                }
            }

            // 7. Phase 4: MeshCollider Recovery & Convex Enforcement
            var colliders = robotRoot.GetComponentsInChildren<MeshCollider>(true);
            foreach (var mc in colliders)
            {
                if (mc.sharedMesh == null)
                {
                    // 1. First search the link's Visuals or siblings for an authentic CAD mesh
                    var linkTransform = mc.GetComponentInParent<Unity.Robotics.UrdfImporter.UrdfLink>()?.transform
                                        ?? (mc.transform.parent != null && mc.transform.parent.name == "Collisions" ? mc.transform.parent.parent : mc.transform.parent);

                    Mesh authenticMesh = null;
                    if (linkTransform != null)
                    {
                        var visualFilters = linkTransform.GetComponentsInChildren<MeshFilter>(true);
                        foreach (var vf in visualFilters)
                        {
                            if (vf != null && vf.sharedMesh != null && !vf.sharedMesh.name.Contains("fallback"))
                            {
                                authenticMesh = vf.sharedMesh;
                                break;
                            }
                        }
                    }

                    if (authenticMesh != null)
                    {
                        mc.sharedMesh = authenticMesh;
                    }
                    else
                    {
                        string linkName = linkTransform != null ? linkTransform.name : mc.name;
                        Mesh recovered = null;
                        if (!string.IsNullOrEmpty(linkName) && linkVisuals.TryGetValue(linkName, out var vList) && vList.Count > 0)
                        {
                            var vItem = vList.FirstOrDefault(x => !string.IsNullOrEmpty(x.meshFile));
                            if (vItem != null)
                            {
                                recovered = ResolveMeshAsset(Path.GetFileName(vItem.meshFile), linkName, localMeshesDir);
                            }
                        }

                        if (recovered == null)
                        {
                            recovered = ResolveMeshAsset(mc.name, linkName, localMeshesDir);
                        }

                        if (recovered != null)
                        {
                            mc.sharedMesh = recovered;
                        }
                        else
                        {
                            // 3. Low-poly procedural fallback guaranteed <= 256 triangles for PhysX convex hull
                            mc.sharedMesh = CreateProceduralFallbackForLink(linkName, GeometryTypes.Mesh);
                        }
                    }
                }
                mc.convex = true; // Mandatory for PhysX ArticulationBody
            }

            // 8. Phase 5: Universal Material & Multi-Submesh Pipeline Upgrade
            var renderers = robotRoot.GetComponentsInChildren<MeshRenderer>(true);
            foreach (var mr in renderers)
            {
                var mf = mr.GetComponent<MeshFilter>() ?? mr.GetComponentInChildren<MeshFilter>();
                int requiredSubmeshes = 1;
                if (mf != null && mf.sharedMesh != null)
                {
                    requiredSubmeshes = Mathf.Max(1, mf.sharedMesh.subMeshCount);
                }

                Material[] currentMaterials = mr.sharedMaterials ?? new Material[0];
                Material[] newMaterials = new Material[requiredSubmeshes];

                bool anyUpgraded = false;
                for (int i = 0; i < requiredSubmeshes; i++)
                {
                    Material curMat = i < currentMaterials.Length ? currentMaterials[i] : null;
                    bool needsUpgrade = curMat == null ||
                                        curMat.shader == null ||
                                        curMat.shader.name.Contains("Error") ||
                                        curMat.shader.name.Contains("InternalErrorShader") ||
                                        (isURP && curMat.shader.name == "Standard") ||
                                        (isURP && curMat.shader.name.Contains("Legacy")) ||
                                        (!isURP && curMat.shader.name.Contains("Universal"));

                    if (needsUpgrade)
                    {
                        string linkName = mr.GetComponentInParent<UrdfLink>()?.name ?? mr.name;
                        newMaterials[i] = CreateHealedMaterial(linkName, curMat, targetShader, isURP, globalMaterials, linkVisuals, localMeshesDir, i);
                        anyUpgraded = true;
                        healedMaterials++;
                    }
                    else
                    {
                        newMaterials[i] = curMat;
                    }
                }

                if (anyUpgraded || currentMaterials.Length != requiredSubmeshes)
                {
                    mr.sharedMaterials = newMaterials;
                }
            }

            AssetDatabase.SaveAssets();

            // Ensure visual and collision meshes match the robot's chosenAxis orientation
            var urdfRobot = robotRoot.GetComponent<UrdfRobot>();
            if (urdfRobot != null && !urdfRobot.CheckOrientation())
            {
                UrdfRobotExtensions.CorrectAxis(robotRoot);
            }

            // 9. Phase 6: Universal Multi-Joint Articulation Stabilization & Anti-Explosion Engine
            int ignoredCollisions = StabilizeArticulationPhysics(robotRoot);

            Debug.Log($"<color=green>[Universal Robot Healer]</color> '{robotRoot.name}' successfully healed & stabilized: " +
                      $"{healedMeshes} mesh(es) verified/restored, {healedMaterials} material(s) upgraded for {targetShader.name}, {ignoredCollisions} internal collision pair(s) ignored.");
        }

        private static Material CreateHealedMaterial(
            string linkName,
            Material existingMat,
            Shader targetShader,
            bool isURP,
            Dictionary<string, UrdfMaterialData> globalMaterials,
            Dictionary<string, List<UrdfVisualData>> linkVisuals,
            string localMeshesDir,
            int submeshIndex = 0)
        {
            Color chosenColor = Color.gray;
            Texture chosenTex = null;
            string materialBaseName = $"{linkName}_Mat_{submeshIndex}";

            // 1. Check URDF XML data for exact defined color & texture
            bool foundInUrdf = false;
            if (linkVisuals.TryGetValue(linkName, out var vList) && vList.Count > 0)
            {
                var vItem = (submeshIndex >= 0 && submeshIndex < vList.Count) ? vList[submeshIndex] : vList[0];
                if (!string.IsNullOrEmpty(vItem.materialName))
                {
                    materialBaseName = vItem.materialName;
                    if (globalMaterials.TryGetValue(vItem.materialName, out var gm))
                    {
                        if (gm.color.HasValue)
                        {
                            chosenColor = gm.color.Value;
                            foundInUrdf = true;
                        }
                        if (!string.IsNullOrEmpty(gm.texturePath))
                        {
                            chosenTex = LoadTextureFromDiskOrProject(gm.texturePath, localMeshesDir);
                        }
                    }
                }

                if (vItem.inlineColor.HasValue)
                {
                    chosenColor = vItem.inlineColor.Value;
                    foundInUrdf = true;
                }
                if (!string.IsNullOrEmpty(vItem.inlineTexture) && chosenTex == null)
                {
                    chosenTex = LoadTextureFromDiskOrProject(vItem.inlineTexture, localMeshesDir);
                }
            }

            // Check in-memory cache first
            if (_healedMaterialCache.TryGetValue(materialBaseName, out var cachedMat) && cachedMat != null)
            {
                return cachedMat;
            }

            // 2. If not defined in URDF, inspect existing material properties
            if (!foundInUrdf && existingMat != null)
            {
                if (existingMat.HasProperty("_BaseColor")) chosenColor = existingMat.GetColor("_BaseColor");
                else if (existingMat.HasProperty("_Color")) chosenColor = existingMat.GetColor("_Color");

                if (chosenTex == null) chosenTex = existingMat.mainTexture;
                materialBaseName = existingMat.name.Replace("_URP", "");
            }

            // 3. Heuristic color for completely black, clear, or undefined links
            string lLower = linkName.ToLowerInvariant();
            if (chosenColor == Color.clear || (chosenColor == Color.black && !lLower.Contains("black") && !lLower.Contains("tire")))
            {
                if (lLower.Contains("wheel") || lLower.Contains("tire")) chosenColor = new Color(0.18f, 0.18f, 0.18f, 1.0f);
                else if (lLower.Contains("chassis") || lLower.Contains("base") || lLower.Contains("body")) chosenColor = new Color(0.85f, 0.75f, 0.2f, 1.0f);
                else if (lLower.Contains("rocker") || lLower.Contains("bogie") || lLower.Contains("suspension")) chosenColor = new Color(0.35f, 0.35f, 0.38f, 1.0f);
                else chosenColor = new Color(0.6f, 0.6f, 0.65f, 1.0f);
            }

            // 4. If texture still null, search folder for matching image
            if (chosenTex == null)
            {
                chosenTex = FindMatchingTexture(linkName, materialBaseName, localMeshesDir);
            }

            // 5. Construct upgraded material asset
            Material mat = new Material(targetShader);
            mat.name = $"{materialBaseName}{(isURP ? "_URP" : "")}";

            if (mat.HasProperty("_BaseColor")) mat.SetColor("_BaseColor", chosenColor);
            if (mat.HasProperty("_Color")) mat.SetColor("_Color", chosenColor);

            if (chosenTex != null)
            {
                if (mat.HasProperty("_BaseMap")) mat.SetTexture("_BaseMap", chosenTex);
                if (mat.HasProperty("_MainTex")) mat.SetTexture("_MainTex", chosenTex);
            }

            if (mat.HasProperty("_Smoothness")) mat.SetFloat("_Smoothness", 0.55f);
            if (mat.HasProperty("_Glossiness")) mat.SetFloat("_Glossiness", 0.55f);
            if (mat.HasProperty("_Metallic")) mat.SetFloat("_Metallic", 0.25f);

            // Save material asset to Materials folder if project-relative
            if (!string.IsNullOrEmpty(localMeshesDir))
            {
                string matsDir = Path.Combine(localMeshesDir, "Materials").Replace('\\', '/');
                if (!Directory.Exists(matsDir)) Directory.CreateDirectory(matsDir);

                string assetPath = GetProjectRelativePath(Path.Combine(matsDir, $"{mat.name}.mat"));
                if (assetPath.StartsWith("Assets", StringComparison.OrdinalIgnoreCase))
                {
                    var existingAsset = AssetDatabase.LoadAssetAtPath<Material>(assetPath);
                    if (existingAsset != null)
                    {
                        EditorUtility.CopySerialized(mat, existingAsset);
                        _healedMaterialCache[materialBaseName] = existingAsset;
                        return existingAsset;
                    }
                    else
                    {
                        AssetDatabase.CreateAsset(mat, assetPath);
                        _healedMaterialCache[materialBaseName] = mat;
                        return mat;
                    }
                }
            }

            _healedMaterialCache[materialBaseName] = mat;
            return mat;
        }

        private static Texture LoadTextureFromDiskOrProject(string texturePath, string localMeshesDir)
        {
            if (string.IsNullOrEmpty(texturePath)) return null;

            string cleanName = Path.GetFileName(texturePath);
            if (_healedTextureCache.TryGetValue(cleanName, out var cachedTex) && cachedTex != null)
            {
                return cachedTex;
            }

            // 1. Check local meshes dir & textures subfolder
            if (!string.IsNullOrEmpty(localMeshesDir) && Directory.Exists(localMeshesDir))
            {
                string direct = Path.Combine(localMeshesDir, cleanName);
                if (File.Exists(direct))
                {
                    var t = LoadTextureAsset(direct);
                    if (t != null) { _healedTextureCache[cleanName] = t; return t; }
                }

                string inTextures = Path.Combine(localMeshesDir, "textures", cleanName);
                if (File.Exists(inTextures))
                {
                    var t = LoadTextureAsset(inTextures);
                    if (t != null) { _healedTextureCache[cleanName] = t; return t; }
                }

                string inMaterials = Path.Combine(localMeshesDir, "materials", "textures", cleanName);
                if (File.Exists(inMaterials))
                {
                    var t = LoadTextureAsset(inMaterials);
                    if (t != null) { _healedTextureCache[cleanName] = t; return t; }
                }
            }

            // 2. Search project AssetDatabase
            string baseName = Path.GetFileNameWithoutExtension(cleanName);
            if (baseName.Length > 2)
            {
                string[] guids = AssetDatabase.FindAssets($"{baseName} t:Texture2D");
                if (guids.Length > 0)
                {
                    string path = AssetDatabase.GUIDToAssetPath(guids[0]);
                    var t = AssetDatabase.LoadAssetAtPath<Texture2D>(path);
                    if (t != null) { _healedTextureCache[cleanName] = t; return t; }
                }
            }

            return null;
        }

        private static Texture FindMatchingTexture(string linkName, string matName, string localMeshesDir)
        {
            if (string.IsNullOrEmpty(localMeshesDir) || !Directory.Exists(localMeshesDir)) return null;

            string[] searchTerms = new string[] { matName, linkName };
            string[] extensions = new string[] { ".png", ".jpg", ".jpeg", ".tga" };

            foreach (var term in searchTerms)
            {
                if (string.IsNullOrEmpty(term)) continue;
                foreach (var ext in extensions)
                {
                    string candidate = Path.Combine(localMeshesDir, term + ext);
                    if (File.Exists(candidate)) return LoadTextureAsset(candidate);

                    string candidateTex = Path.Combine(localMeshesDir, "textures", term + ext);
                    if (File.Exists(candidateTex)) return LoadTextureAsset(candidateTex);
                }
            }

            return null;
        }

        private static Texture LoadTextureAsset(string filePath)
        {
            string rel = GetProjectRelativePath(filePath);
            if (rel.StartsWith("Assets", StringComparison.OrdinalIgnoreCase))
            {
                return AssetDatabase.LoadAssetAtPath<Texture2D>(rel);
            }
            return null;
        }

        private static Mesh ResolveMeshAsset(string objectName, string parentName, string localMeshesDir)
        {
            string cleanName = objectName;
            int lastUnder = cleanName.LastIndexOf('_');
            if (lastUnder > 0 && int.TryParse(cleanName.Substring(lastUnder + 1), out _))
            {
                cleanName = cleanName.Substring(0, lastUnder);
            }

            string cacheKey = $"{objectName}_{parentName}";
            if (_healedMeshCache.TryGetValue(cacheKey, out var cachedM) && cachedM != null)
            {
                return cachedM;
            }

            // 1. Check local meshes directory and subdirectories if specified
            if (!string.IsNullOrEmpty(localMeshesDir) && Directory.Exists(localMeshesDir))
            {
                var searchDirs = new List<string> { localMeshesDir };
                string subMeshes = Path.Combine(localMeshesDir, "meshes");
                if (Directory.Exists(subMeshes)) searchDirs.Add(subMeshes);
                string parentDir = Path.GetDirectoryName(localMeshesDir);
                if (!string.IsNullOrEmpty(parentDir) && Directory.Exists(parentDir))
                {
                    string parentMeshes = Path.Combine(parentDir, "meshes");
                    if (Directory.Exists(parentMeshes) && !searchDirs.Contains(parentMeshes)) searchDirs.Add(parentMeshes);
                }

                foreach (var dir in searchDirs)
                {
                    string[] candidates = new string[]
                    {
                        Path.Combine(dir, $"{objectName}.asset"),
                        Path.Combine(dir, $"{cleanName}_0.asset"),
                        Path.Combine(dir, $"{cleanName}.stl"),
                        Path.Combine(dir, $"{cleanName}.STL"),
                        Path.Combine(dir, $"{cleanName}.dae"),
                        Path.Combine(dir, $"{cleanName}.obj"),
                        Path.Combine(dir, $"{cleanName}.fbx"),
                        Path.Combine(dir, $"{cleanName}.prefab"),
                        Path.Combine(dir, $"{parentName}_0.asset"),
                        Path.Combine(dir, $"{parentName}.stl"),
                        Path.Combine(dir, $"{parentName}.STL"),
                        Path.Combine(dir, $"{parentName}.dae"),
                        Path.Combine(dir, $"{parentName}.obj"),
                        Path.Combine(dir, $"{parentName}.fbx"),
                        Path.Combine(dir, $"{parentName}.prefab")
                    };

                    foreach (var c in candidates)
                    {
                        if (File.Exists(c))
                        {
                            var m = LoadMeshFromFilePath(c.Replace('\\', '/'));
                            if (m != null)
                            {
                                _healedMeshCache[cacheKey] = m;
                                return m;
                            }
                        }
                    }
                }
            }

            // 2. Search robot local directory via AssetDatabase with selective non-generic terms
            string[] searchFolders = null;
            if (!string.IsNullOrEmpty(localMeshesDir) && Directory.Exists(localMeshesDir))
            {
                string parentDir = Path.GetDirectoryName(Path.GetFullPath(localMeshesDir).TrimEnd('/', '\\'));
                if (!string.IsNullOrEmpty(parentDir))
                {
                    string relFolder = GetProjectRelativePath(parentDir);
                    if (!string.IsNullOrEmpty(relFolder) && relFolder.StartsWith("Assets", StringComparison.OrdinalIgnoreCase))
                    {
                        searchFolders = new string[] { relFolder };
                    }
                }
            }

            string[] searchTerms = new string[] { objectName, cleanName, parentName };
            foreach (var term in searchTerms)
            {
                if (string.IsNullOrEmpty(term) || term.Length <= 2 ||
                    term.Equals("link", StringComparison.OrdinalIgnoreCase) ||
                    term.Equals("unnamed", StringComparison.OrdinalIgnoreCase) ||
                    term.Equals("collision", StringComparison.OrdinalIgnoreCase) ||
                    term.Equals("visual", StringComparison.OrdinalIgnoreCase)) continue;

                string[] guids = searchFolders != null ? AssetDatabase.FindAssets($"{term} t:Mesh", searchFolders) : new string[0];
                if (guids.Length == 0 && searchFolders != null)
                {
                    guids = AssetDatabase.FindAssets(term, searchFolders);
                }
                foreach (var g in guids)
                {
                    string assetPath = AssetDatabase.GUIDToAssetPath(g).Replace('\\', '/');
                    if (assetPath.EndsWith(".asset", StringComparison.OrdinalIgnoreCase) ||
                        assetPath.EndsWith(".stl", StringComparison.OrdinalIgnoreCase) ||
                        assetPath.EndsWith(".dae", StringComparison.OrdinalIgnoreCase) ||
                        assetPath.EndsWith(".obj", StringComparison.OrdinalIgnoreCase) ||
                        assetPath.EndsWith(".fbx", StringComparison.OrdinalIgnoreCase) ||
                        assetPath.EndsWith(".prefab", StringComparison.OrdinalIgnoreCase))
                    {
                        var m = LoadMeshFromFilePath(assetPath);
                        if (m != null)
                        {
                            _healedMeshCache[cacheKey] = m;
                            return m;
                        }
                    }
                }
            }

            return null;
        }

        private static Mesh LoadMeshFromFilePath(string assetPath)
        {
            try
            {
                string projectRelative = GetProjectRelativePath(assetPath);
                if (projectRelative.EndsWith(".asset", StringComparison.OrdinalIgnoreCase))
                {
                    return AssetDatabase.LoadAssetAtPath<Mesh>(projectRelative);
                }

                if (projectRelative.EndsWith(".stl", StringComparison.OrdinalIgnoreCase))
                {
                    string diskPath = Path.GetFullPath(projectRelative);
                    var meshes = StlImporter.ImportMesh(diskPath);
                    if (meshes != null && meshes.Length > 0)
                    {
                        string outAsset = projectRelative.Substring(0, projectRelative.Length - 4) + "_0.asset";
                        if (!File.Exists(outAsset))
                        {
                            AssetDatabase.CreateAsset(meshes[0], outAsset);
                            AssetDatabase.SaveAssets();
                        }
                        return meshes[0];
                    }
                }

                if (projectRelative.EndsWith(".prefab", StringComparison.OrdinalIgnoreCase))
                {
                    var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(projectRelative);
                    if (prefab != null)
                    {
                        var mf = prefab.GetComponentInChildren<MeshFilter>(true);
                        if (mf != null && mf.sharedMesh != null) return mf.sharedMesh;
                    }
                }

                // DAE, OBJ, FBX
                var allObjs = AssetDatabase.LoadAllAssetsAtPath(projectRelative);
                foreach (var o in allObjs)
                {
                    if (o is Mesh m) return m;
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[Rover Importer] Failed to load mesh from '{assetPath}': {ex.Message}");
            }

            return null;
        }

        private static Mesh CreateProceduralFallbackForLink(string linkName, GeometryTypes geomType)
        {
            string lLower = linkName.ToLowerInvariant();
            bool isWheel = lLower.Contains("wheel") || lLower.Contains("tire") || geomType == GeometryTypes.Cylinder;
            bool isChassis = lLower.Contains("base") || lLower.Contains("chassis") || lLower.Contains("body") || geomType == GeometryTypes.Box;

            if (isWheel)
            {
                var m = GenerateProceduralCylinder(0.26f, 0.22f);
                m.name = $"{linkName}_cylinder_fallback";
                return m;
            }
            else if (isChassis)
            {
                var m = GenerateProceduralBox(new Vector3(0.6f, 0.35f, 0.85f));
                m.name = $"{linkName}_box_fallback";
                return m;
            }
            else
            {
                // Strict PhysX convex mesh budget: <= 256 triangles (8x8 segments = 128 triangles)
                var m = GenerateProceduralSphere(0.08f, 8, 8);
                m.name = $"{linkName}_sphere_fallback";
                return m;
            }
        }

        private static void ParseUrdfData(
            string urdfPath,
            out Dictionary<string, UrdfMaterialData> materials,
            out Dictionary<string, List<UrdfVisualData>> linkVisuals,
            out List<UrdfLinkData> links)
        {
            materials = new Dictionary<string, UrdfMaterialData>(StringComparer.OrdinalIgnoreCase);
            linkVisuals = new Dictionary<string, List<UrdfVisualData>>(StringComparer.OrdinalIgnoreCase);
            links = new List<UrdfLinkData>();

            if (string.IsNullOrEmpty(urdfPath) || !File.Exists(urdfPath)) return;

            try
            {
                XDocument doc = XDocument.Load(urdfPath);
                var root = doc.Root;
                if (root == null) return;

                // 1. Global materials
                foreach (var m in root.Elements("material"))
                {
                    string mName = m.Attribute("name")?.Value;
                    if (string.IsNullOrEmpty(mName)) continue;

                    var data = new UrdfMaterialData { name = mName };
                    var colElem = m.Element("color");
                    if (colElem != null)
                    {
                        data.color = ParseColor(colElem.Attribute("rgba")?.Value);
                    }
                    var texElem = m.Element("texture");
                    if (texElem != null)
                    {
                        data.texturePath = texElem.Attribute("filename")?.Value;
                    }
                    materials[mName] = data;
                }

                // 2. Links & Visuals
                foreach (var l in root.Elements("link"))
                {
                    string lName = l.Attribute("name")?.Value;
                    if (string.IsNullOrEmpty(lName)) continue;

                    var linkObj = new UrdfLinkData { name = lName };
                    links.Add(linkObj);

                    var visualsList = new List<UrdfVisualData>();
                    foreach (var v in l.Elements("visual"))
                    {
                        var vData = new UrdfVisualData { linkName = lName, visualName = v.Attribute("name")?.Value };
                        var geom = v.Element("geometry");
                        if (geom != null)
                        {
                            var mesh = geom.Element("mesh");
                            if (mesh != null)
                            {
                                vData.geometryType = "mesh";
                                vData.meshFile = mesh.Attribute("filename")?.Value;
                                vData.scale = ParseVector3(mesh.Attribute("scale")?.Value);
                            }
                            else if (geom.Element("box") != null) vData.geometryType = "box";
                            else if (geom.Element("cylinder") != null) vData.geometryType = "cylinder";
                            else if (geom.Element("sphere") != null) vData.geometryType = "sphere";
                        }

                        var mat = v.Element("material");
                        if (mat != null)
                        {
                            vData.materialName = mat.Attribute("name")?.Value;
                            var cElem = mat.Element("color");
                            if (cElem != null) vData.inlineColor = ParseColor(cElem.Attribute("rgba")?.Value);
                            var tElem = mat.Element("texture");
                            if (tElem != null) vData.inlineTexture = tElem.Attribute("filename")?.Value;
                        }
                        visualsList.Add(vData);
                    }
                    linkVisuals[lName] = visualsList;
                }

                // 3. Joints (for parent/child hierarchy)
                foreach (var j in root.Elements("joint"))
                {
                    string jName = j.Attribute("name")?.Value;
                    string jType = j.Attribute("type")?.Value;
                    string parent = j.Element("parent")?.Attribute("link")?.Value;
                    string child = j.Element("child")?.Attribute("link")?.Value;

                    var targetLink = links.FirstOrDefault(x => string.Equals(x.name, child, StringComparison.OrdinalIgnoreCase));
                    if (targetLink != null)
                    {
                        targetLink.parentJoint = jName;
                        targetLink.parentLink = parent;
                        targetLink.jointType = jType;
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[Rover Model Healer] Notice reading URDF metadata: {ex.Message}");
            }
        }

        private static Color? ParseColor(string rgba)
        {
            if (string.IsNullOrEmpty(rgba)) return null;
            var parts = rgba.Split(new[] { ' ', '\t', ',' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length >= 4 &&
                float.TryParse(parts[0], NumberStyles.Any, CultureInfo.InvariantCulture, out float r) &&
                float.TryParse(parts[1], NumberStyles.Any, CultureInfo.InvariantCulture, out float g) &&
                float.TryParse(parts[2], NumberStyles.Any, CultureInfo.InvariantCulture, out float b) &&
                float.TryParse(parts[3], NumberStyles.Any, CultureInfo.InvariantCulture, out float a))
            {
                return new Color(r, g, b, a);
            }
            return null;
        }

        private static Vector3? ParseVector3(string vec)
        {
            if (string.IsNullOrEmpty(vec)) return null;
            var parts = vec.Split(new[] { ' ', '\t', ',' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length >= 3 &&
                float.TryParse(parts[0], NumberStyles.Any, CultureInfo.InvariantCulture, out float x) &&
                float.TryParse(parts[1], NumberStyles.Any, CultureInfo.InvariantCulture, out float y) &&
                float.TryParse(parts[2], NumberStyles.Any, CultureInfo.InvariantCulture, out float z))
            {
                return new Vector3(x, y, z);
            }
            return null;
        }

        private static string FindMeshesDirectoryForRobot(string robotName)
        {
            string[] matchingDirs = Directory.GetDirectories(Application.dataPath, "meshes", SearchOption.AllDirectories);
            foreach (var d in matchingDirs)
            {
                string norm = d.Replace('\\', '/');
                if (norm.IndexOf(robotName, StringComparison.OrdinalIgnoreCase) >= 0)
                {
                    return norm;
                }
            }
            return matchingDirs.Length > 0 ? matchingDirs[0].Replace('\\', '/') : null;
        }

        private static string FindUrdfFileForRobot(string robotName)
        {
            var urdfs = Directory.GetFiles(Application.dataPath, "*.urdf", SearchOption.AllDirectories);
            foreach (var u in urdfs)
            {
                string norm = u.Replace('\\', '/');
                if (norm.IndexOf(robotName, StringComparison.OrdinalIgnoreCase) >= 0)
                {
                    return norm;
                }
            }
            return null;
        }

        private static string GetProjectRelativePath(string fullPath)
        {
            string projectPath = Path.GetFullPath(Application.dataPath + "/..").Replace('\\', '/');
            string cleanFullPath = Path.GetFullPath(fullPath).Replace('\\', '/');
            if (cleanFullPath.StartsWith(projectPath, StringComparison.OrdinalIgnoreCase))
            {
                return cleanFullPath.Substring(projectPath.Length + 1);
            }
            return fullPath;
        }

        #region Procedural Fallback Mesh Generators
        private static Mesh GenerateProceduralCylinder(float radius, float height, int segments = 24)
        {
            var mesh = new Mesh { name = "ProceduralCylinder" };
            var vertices = new List<Vector3>();
            var triangles = new List<int>();
            var normals = new List<Vector3>();
            var uvs = new List<Vector2>();

            float halfHeight = height * 0.5f;

            for (int i = 0; i <= segments; i++)
            {
                float angle = (float)i / segments * Mathf.PI * 2f;
                float x = Mathf.Cos(angle) * radius;
                float z = Mathf.Sin(angle) * radius;

                vertices.Add(new Vector3(x, -halfHeight, z));
                vertices.Add(new Vector3(x, halfHeight, z));

                Vector3 normal = new Vector3(x, 0, z).normalized;
                normals.Add(normal);
                normals.Add(normal);

                float u = (float)i / segments;
                uvs.Add(new Vector2(u, 0));
                uvs.Add(new Vector2(u, 1));
            }

            for (int i = 0; i < segments; i++)
            {
                int baseIndex = i * 2;
                triangles.Add(baseIndex);
                triangles.Add(baseIndex + 1);
                triangles.Add(baseIndex + 2);

                triangles.Add(baseIndex + 2);
                triangles.Add(baseIndex + 1);
                triangles.Add(baseIndex + 3);
            }

            mesh.SetVertices(vertices);
            mesh.SetNormals(normals);
            mesh.SetUVs(0, uvs);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateBounds();
            return mesh;
        }

        private static Mesh GenerateProceduralBox(Vector3 size)
        {
            var mesh = new Mesh { name = "ProceduralBox" };
            var h = size * 0.5f;

            Vector3[] vertices = new Vector3[]
            {
                // Front
                new Vector3(-h.x, -h.y,  h.z), new Vector3( h.x, -h.y,  h.z),
                new Vector3( h.x,  h.y,  h.z), new Vector3(-h.x,  h.y,  h.z),
                // Back
                new Vector3( h.x, -h.y, -h.z), new Vector3(-h.x, -h.y, -h.z),
                new Vector3(-h.x,  h.y, -h.z), new Vector3( h.x,  h.y, -h.z),
                // Top
                new Vector3(-h.x,  h.y,  h.z), new Vector3( h.x,  h.y,  h.z),
                new Vector3( h.x,  h.y, -h.z), new Vector3(-h.x,  h.y, -h.z),
                // Bottom
                new Vector3(-h.x, -h.y, -h.z), new Vector3( h.x, -h.y, -h.z),
                new Vector3( h.x, -h.y,  h.z), new Vector3(-h.x, -h.y,  h.z),
                // Left
                new Vector3(-h.x, -h.y, -h.z), new Vector3(-h.x, -h.y,  h.z),
                new Vector3(-h.x,  h.y,  h.z), new Vector3(-h.x,  h.y, -h.z),
                // Right
                new Vector3( h.x, -h.y,  h.z), new Vector3( h.x, -h.y, -h.z),
                new Vector3( h.x,  h.y, -h.z), new Vector3( h.x,  h.y,  h.z),
            };

            int[] triangles = new int[]
            {
                0,2,1, 0,3,2,
                4,6,5, 4,7,6,
                8,10,9, 8,11,10,
                12,14,13, 12,15,14,
                16,18,17, 16,19,18,
                20,22,21, 20,23,22
            };

            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        private static Mesh GenerateProceduralSphere(float radius, int latSegments = 8, int lonSegments = 8)
        {
            var mesh = new Mesh { name = "ProceduralSphere" };
            var vertices = new List<Vector3>();
            var triangles = new List<int>();
            var normals = new List<Vector3>();

            for (int lat = 0; lat <= latSegments; lat++)
            {
                float theta = lat * Mathf.PI / latSegments;
                float sinTheta = Mathf.Sin(theta);
                float cosTheta = Mathf.Cos(theta);

                for (int lon = 0; lon <= lonSegments; lon++)
                {
                    float phi = lon * 2f * Mathf.PI / lonSegments;
                    float sinPhi = Mathf.Sin(phi);
                    float cosPhi = Mathf.Cos(phi);

                    Vector3 normal = new Vector3(cosPhi * sinTheta, cosTheta, sinPhi * sinTheta);
                    normals.Add(normal);
                    vertices.Add(normal * radius);
                }
            }

            for (int lat = 0; lat < latSegments; lat++)
            {
                for (int lon = 0; lon < lonSegments; lon++)
                {
                    int first = lat * (lonSegments + 1) + lon;
                    int second = first + lonSegments + 1;

                    triangles.Add(first);
                    triangles.Add(second);
                    triangles.Add(first + 1);

                    triangles.Add(second);
                    triangles.Add(second + 1);
                    triangles.Add(first + 1);
                }
            }

            mesh.SetVertices(vertices);
            mesh.SetNormals(normals);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateBounds();
            return mesh;
        }
        #endregion

        #region Multi-Joint Articulation Stabilization
        /// <summary>
        /// Solves the classic PhysX ArticulationBody collapse / explosive destruction problem in complex multi-joint robots:
        /// 1. Self-Collision Explosions: Ignores collisions between all internal colliders of the robot so overlapping
        ///    pivots never generate catastrophic separation impulses.
        /// 2. Numerical Convergence: Upgrades solverIterations to 24, velocity iterations to 8, and caps depenetration velocity.
        /// 3. Constraint Drift & Detachment: Enforces locked linear DOFs on all joints and locks all DOFs on FixedJoints.
        /// 4. Limb Collapse: Assigns calibrated holding stiffness and damping to non-wheel revolute joints (legs, arms, neck)
        ///    so the robot maintains its structural pose instead of crumpling under gravity.
        /// 5. Mass Ratio Balancing (PhysX 20:1 rule): Normalizes micro-mass links (<0.05kg) to prevent numerical divergence.
        /// </summary>
        public static int StabilizeArticulationPhysics(GameObject robotRoot)
        {
            if (robotRoot == null) return 0;

            // 1. Ignore intra-robot collisions between all internal colliders
            var colliders = robotRoot.GetComponentsInChildren<Collider>(true);
            int ignoredPairs = 0;
            for (int i = 0; i < colliders.Length; i++)
            {
                for (int j = i + 1; j < colliders.Length; j++)
                {
                    Physics.IgnoreCollision(colliders[i], colliders[j], true);
                    ignoredPairs++;
                }
            }

            // 2. Tune all ArticulationBodies in the hierarchy
            var bodies = robotRoot.GetComponentsInChildren<ArticulationBody>(true);
            if (bodies == null || bodies.Length == 0) return ignoredPairs;

            var rootBody = bodies.FirstOrDefault(b => b.isRoot) ?? bodies[0];
            rootBody.immovable = false;
            rootBody.useGravity = true;

            foreach (var body in bodies)
            {
                if (body == null) continue;

                // High-precision solver settings to handle 10-30+ DOFs
                body.solverIterations = 24;
                body.solverVelocityIterations = 8;
                body.maxDepenetrationVelocity = 2.0f; // Critical: Prevents explosive catapult launches

                // Prevent micro-mass divergence (PhysX 20:1 mass ratio instability)
                if (body.mass < 0.05f)
                {
                    body.mass = 0.05f;
                }

                // Ensure valid non-zero diagonal inertia tensor
                if (body.inertiaTensor.x < 1e-4f || body.inertiaTensor.y < 1e-4f || body.inertiaTensor.z < 1e-4f)
                {
                    float minInertia = Mathf.Max(0.001f, body.mass * 0.005f);
                    body.inertiaTensor = new Vector3(
                        Mathf.Max(body.inertiaTensor.x, minInertia),
                        Mathf.Max(body.inertiaTensor.y, minInertia),
                        Mathf.Max(body.inertiaTensor.z, minInertia)
                    );
                }

                if (body.jointType == ArticulationJointType.FixedJoint)
                {
                    // Fixed joints must NEVER have free linear degrees of freedom
                    body.linearLockX = ArticulationDofLock.LockedMotion;
                    body.linearLockY = ArticulationDofLock.LockedMotion;
                    body.linearLockZ = ArticulationDofLock.LockedMotion;
                    body.twistLock = ArticulationDofLock.LockedMotion;
                    body.swingYLock = ArticulationDofLock.LockedMotion;
                    body.swingZLock = ArticulationDofLock.LockedMotion;
                }
                else if (body.jointType == ArticulationJointType.RevoluteJoint)
                {
                    // Linear axes must always be locked on revolute joints to prevent joint stretching
                    body.linearLockX = ArticulationDofLock.LockedMotion;
                    body.linearLockY = ArticulationDofLock.LockedMotion;
                    body.linearLockZ = ArticulationDofLock.LockedMotion;

                    // Revolute hinges MUST have non-driving rotation axes locked to prevent 3D dislocation
                    body.swingYLock = ArticulationDofLock.LockedMotion;
                    body.swingZLock = ArticulationDofLock.LockedMotion;

                    // Check if this is a wheel or an articulated limb/arm/leg
                    string nameLower = body.name.ToLowerInvariant();
                    bool isWheel = nameLower.Contains("wheel") || nameLower.Contains("tire") || nameLower.Contains("tyre");

                    if (!isWheel)
                    {
                        // Articulated limb (leg, arm, neck, steer): provide posture-holding drive
                        // so gravity doesn't collapse the robot into a puddle
                        var drive = body.xDrive;
                        float massScale = Mathf.Max(0.5f, body.mass);

                        drive.driveType = ArticulationDriveType.Target;
                        drive.stiffness = Mathf.Max(1500f, massScale * 3500f);
                        drive.damping = Mathf.Max(120f, massScale * 200f);
                        drive.forceLimit = Mathf.Max(3000f, massScale * 6000f);
                        drive.target = Mathf.Clamp(drive.target, drive.lowerLimit, drive.upperLimit);
                        body.xDrive = drive;
                    }
                }
            }

            // Ensure runtime stabilizer component is attached to persist collision ignoring into Play Mode
            var stabilizer = robotRoot.GetComponent<RobotPhysicsStabilizer>();
            if (stabilizer == null)
            {
                stabilizer = robotRoot.AddComponent<RobotPhysicsStabilizer>();
            }
            stabilizer.ApplyStabilization();

            return ignoredPairs;
        }

        /// <summary>
        /// Measures the lowest point of all colliders in the robot hierarchy and smoothly
        /// offsets the root GameObject so its feet/wheels rest flush with the floor plane (Y = 0)
        /// or the underlying terrain/mesh without interpenetrating.
        /// </summary>
        public static float GroundRobotOnFloor(GameObject robotRoot, float targetFloorY = 0f)
        {
            if (robotRoot == null) return 0f;

            Physics.SyncTransforms();

            var colliders = robotRoot.GetComponentsInChildren<Collider>(true);
            if (colliders == null || colliders.Length == 0) return 0f;

            float lowestY = float.MaxValue;
            foreach (var col in colliders)
            {
                if (col == null || !col.enabled || col.isTrigger) continue;
                lowestY = Mathf.Min(lowestY, col.bounds.min.y);
            }

            if (lowestY < float.MaxValue - 1f)
            {
                float deltaY = targetFloorY - lowestY;
                if (Mathf.Abs(deltaY) > 0.005f)
                {
                    Undo.RecordObject(robotRoot.transform, "Ground Robot on Floor");
                    robotRoot.transform.position += new Vector3(0f, deltaY, 0f);

                    // If robot contains an ArticulationBody root, teleport its PhysX root to align with transform
                    var rootAb = robotRoot.GetComponentsInChildren<ArticulationBody>(true).FirstOrDefault(b => b.isRoot);
                    if (rootAb != null)
                    {
                        Vector3 targetAbPos = rootAb.transform.position + new Vector3(0f, deltaY, 0f);
                        rootAb.TeleportRoot(targetAbPos, rootAb.transform.rotation);
                    }

                    Physics.SyncTransforms();
                    Debug.Log($"<color=green>[Rover Model Healer]</color> Grounded '{robotRoot.name}': Offset Y by {deltaY:+0.000;-0.000}m to rest flush on floor.");
                    return deltaY;
                }
            }

            return 0f;
        }

        [MenuItem("GameObject/Rover Compatibility/📐 Snap & Ground Robot on Floor", false, 12)]
        public static void GroundSelectedRobot()
        {
            var selected = Selection.activeGameObject;
            if (selected == null)
            {
                EditorUtility.DisplayDialog("Notice", "Select an imported robot in the hierarchy to ground on floor.", "OK");
                return;
            }

            float offset = GroundRobotOnFloor(selected);
            EditorUtility.DisplayDialog("Grounded", $"Offset '{selected.name}' by {offset:F3}m to rest flush on floor.", "OK");
        }

        [MenuItem("GameObject/Rover Compatibility/🦿 Apply Standing Stance Pose", false, 13)]
        public static void ApplyStanceSelectedRobot()
        {
            var selected = Selection.activeGameObject;
            if (selected == null)
            {
                EditorUtility.DisplayDialog("Notice", "Select an articulated robot in the hierarchy to apply standing stance.", "OK");
                return;
            }

            var controller = selected.GetComponent<ArticulatedRobotController>();
            if (controller == null)
            {
                controller = selected.AddComponent<ArticulatedRobotController>();
            }
            controller.InitializeJoints();
            controller.ApplyStandStance();
            EditorUtility.DisplayDialog("Stance Applied", $"Configured standing posture for {controller.joints.Count} joints on '{selected.name}'.", "OK");
        }
        #endregion
    }
}
