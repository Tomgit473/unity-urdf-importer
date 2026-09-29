using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using UnityEditor;
using UnityEngine;
using Unity.Robotics.UrdfImporter;
using UnityMeshImporter;

namespace RoverCompatibility.Editor
{
    public enum MeshResolutionStatus
    {
        DirectlyFound,
        RemappedInProject,
        FormatSubstituted,  // .gltf/.glb remapped to matching .stl/.dae/.obj/.prefab
        FallbackPrimitive,  // Missing mesh replaced with primitive geometry
        Missing
    }

    [Serializable]
    public class MeshPathItem
    {
        public string linkName;
        public string originalUri;
        public string cleanFileName;
        public string resolvedAssetPath;
        public string resolvedRelativePath;
        public MeshResolutionStatus status;
        public bool isCollision;
    }

    public class UrdfAnalysisResult
    {
        public string robotName = "Unknown";
        public string urdfFilePath;
        public int linkCount = 0;
        public int jointCount = 0;
        public List<string> candidateWheelLinks = new List<string>();
        public List<MeshPathItem> meshItems = new List<MeshPathItem>();

        public int foundCount => meshItems.Count(m => m.status == MeshResolutionStatus.DirectlyFound);
        public int remappedCount => meshItems.Count(m => m.status == MeshResolutionStatus.RemappedInProject);
        public int formatSubstitutedCount => meshItems.Count(m => m.status == MeshResolutionStatus.FormatSubstituted);
        public int fallbackPrimitiveCount => meshItems.Count(m => m.status == MeshResolutionStatus.FallbackPrimitive);
        public int missingCount => meshItems.Count(m => m.status == MeshResolutionStatus.Missing);

        public int zeroMassLinksCount = 0;
        public int zeroInertiaLinksCount = 0;
        public int missingWheelColliderCount = 0;

        public ImportSettings.axisType detectedAxis = ImportSettings.axisType.yAxis;
        public string detectedAxisReason = "Y-Axis Up (Standard Unity / DAE / STL)";

        public bool HasGltfMeshes => meshItems.Any(m => m.originalUri.EndsWith(".gltf", StringComparison.OrdinalIgnoreCase) ||
                                                         m.originalUri.EndsWith(".glb", StringComparison.OrdinalIgnoreCase));
        public bool HasZeroMass => zeroMassLinksCount > 0;
        public bool HasMissingMeshes => missingCount > 0;
    }

    /// <summary>
    /// Inspects URDF XML, resolves broken package:// and relative mesh paths,
    /// eliminates ../ path traversals by syncing meshes into a local meshes/ folder,
    /// converts unsupported formats (.gltf/.glb) to compatible project meshes (.stl/.dae/.obj),
    /// generates fallback primitives for missing geometry, auto-heals zero-mass links,
    /// and generates a sanitized, 100% loadable URDF.
    /// </summary>
    public static class UrdfMeshPathResolver
    {
        private static readonly string[] SupportedModelExtensions = { ".stl", ".dae", ".obj", ".prefab", ".fbx", ".gltf", ".glb" };
        private static readonly string[] UnsupportedModelExtensions = { ".bin" };

        public static UrdfAnalysisResult AnalyzeUrdf(string urdfPath, string customMeshDirectory = null)
        {
            var result = new UrdfAnalysisResult { urdfFilePath = urdfPath };
            if (string.IsNullOrEmpty(urdfPath) || !File.Exists(urdfPath))
            {
                return result;
            }

            try
            {
                XDocument doc = XDocument.Load(urdfPath);
                var robotElem = doc.Root;
                if (robotElem != null && robotElem.Name.LocalName == "robot")
                {
                    result.robotName = robotElem.Attribute("name")?.Value ?? "UnnamedRobot";

                    var links = robotElem.Elements("link").ToList();
                    result.linkCount = links.Count;

                    var joints = robotElem.Elements("joint").ToList();
                    result.jointCount = joints.Count;

                    // 1. Identify candidate wheels from joints
                    foreach (var joint in joints)
                    {
                        string jType = joint.Attribute("type")?.Value?.ToLowerInvariant() ?? "";
                        string jName = joint.Attribute("name")?.Value?.ToLowerInvariant() ?? "";
                        string childLink = joint.Element("child")?.Attribute("link")?.Value ?? "";

                        if (jType == "continuous" || jName.Contains("wheel") || jName.Contains("tire") || jName.Contains("tyre") || jName.Contains("track"))
                        {
                            if (!string.IsNullOrEmpty(childLink) && !result.candidateWheelLinks.Contains(childLink))
                            {
                                result.candidateWheelLinks.Add(childLink);
                            }
                        }
                    }

                    // Also check link names directly for wheels
                    foreach (var link in links)
                    {
                        string lName = link.Attribute("name")?.Value ?? "";
                        string lNameLower = lName.ToLowerInvariant();
                        if ((lNameLower.Contains("wheel") || lNameLower.Contains("tire") || lNameLower.Contains("tyre") || lNameLower.Contains("track")) && !result.candidateWheelLinks.Contains(lName))
                        {
                            result.candidateWheelLinks.Add(lName);
                        }
                    }

                    string urdfDirectory = Path.GetDirectoryName(urdfPath);

                    // 2. Scan visual & collision mesh tags
                    foreach (var link in links)
                    {
                        string linkName = link.Attribute("name")?.Value ?? "unknown_link";

                        // Check mass & inertia
                        var massElem = link.Element("inertial")?.Element("mass");
                        double massVal = 0;
                        if (massElem != null)
                        {
                            double.TryParse(massElem.Attribute("value")?.Value, NumberStyles.Any, CultureInfo.InvariantCulture, out massVal);
                        }
                        if (massElem == null || massVal <= 0.0001)
                        {
                            result.zeroMassLinksCount++;
                        }

                        var inertiaElem = link.Element("inertial")?.Element("inertia");
                        double ixx = 0, iyy = 0, izz = 0;
                        if (inertiaElem != null)
                        {
                            double.TryParse(inertiaElem.Attribute("ixx")?.Value, NumberStyles.Any, CultureInfo.InvariantCulture, out ixx);
                            double.TryParse(inertiaElem.Attribute("iyy")?.Value, NumberStyles.Any, CultureInfo.InvariantCulture, out iyy);
                            double.TryParse(inertiaElem.Attribute("izz")?.Value, NumberStyles.Any, CultureInfo.InvariantCulture, out izz);
                        }
                        if (inertiaElem == null || ixx <= 0.00001 || iyy <= 0.00001 || izz <= 0.00001)
                        {
                            result.zeroInertiaLinksCount++;
                        }

                        // Check wheel colliders
                        bool isWheel = result.candidateWheelLinks.Contains(linkName) || linkName.ToLowerInvariant().Contains("wheel");
                        if (isWheel && link.Element("collision") == null)
                        {
                            result.missingWheelColliderCount++;
                        }

                        // Visual meshes
                        foreach (var visual in link.Elements("visual"))
                        {
                            var mesh = visual.Element("geometry")?.Element("mesh");
                            if (mesh != null)
                            {
                                string filename = mesh.Attribute("filename")?.Value;
                                if (!string.IsNullOrEmpty(filename))
                                {
                                    result.meshItems.Add(ResolveMeshItem(linkName, filename, urdfDirectory, customMeshDirectory, false));
                                }
                            }
                        }

                        // Collision meshes
                        foreach (var collision in link.Elements("collision"))
                        {
                            var mesh = collision.Element("geometry")?.Element("mesh");
                            if (mesh != null)
                            {
                                string filename = mesh.Attribute("filename")?.Value;
                                if (!string.IsNullOrEmpty(filename))
                                {
                                    result.meshItems.Add(ResolveMeshItem(linkName, filename, urdfDirectory, customMeshDirectory, true));
                                }
                            }
                        }
                    }
                }

                // Determine optimal coordinate axis:
                // Standard ROS URDF specifications define Z as Up.
                // When meshes are .obj (Wavefront CAD), Unity's ModelImporter does not convert ROS coordinates,
                // so ImportSettings.axisType.zAxis is required to apply the necessary Euler(-90, 0, 90) alignment.
                // When meshes are .dae (Collada), ColladaAssetPostProcessor already rotates them to Unity's Y-up.
                int objCount = result.meshItems.Count(m => (m.originalUri?.EndsWith(".obj", StringComparison.OrdinalIgnoreCase) == true) ||
                                                           (m.resolvedAssetPath?.EndsWith(".obj", StringComparison.OrdinalIgnoreCase) == true));
                int daeCount = result.meshItems.Count(m => (m.originalUri?.EndsWith(".dae", StringComparison.OrdinalIgnoreCase) == true) ||
                                                           (m.resolvedAssetPath?.EndsWith(".dae", StringComparison.OrdinalIgnoreCase) == true));

                if (objCount > 0 && objCount >= daeCount)
                {
                    result.detectedAxis = ImportSettings.axisType.zAxis;
                    result.detectedAxisReason = $"Z-Axis Up (Auto-detected from {objCount} OBJ mesh{(objCount > 1 ? "es" : "")})";
                }
                else if (daeCount > 0)
                {
                    result.detectedAxis = ImportSettings.axisType.yAxis;
                    result.detectedAxisReason = $"Y-Axis Up (Auto-detected from {daeCount} Collada DAE mesh{(daeCount > 1 ? "es" : "")})";
                }
                else
                {
                    result.detectedAxis = ImportSettings.axisType.yAxis;
                    result.detectedAxisReason = "Y-Axis Up (Standard Unity / STL)";
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"[Rover Importer] Error parsing URDF XML at '{urdfPath}': {ex.Message}");
            }

            return result;
        }

        public static ImportSettings.axisType DetectOptimalAxis(string urdfPath)
        {
            if (string.IsNullOrEmpty(urdfPath) || !File.Exists(urdfPath)) return ImportSettings.axisType.yAxis;
            var analysis = AnalyzeUrdf(urdfPath);
            return analysis.detectedAxis;
        }

        private static MeshPathItem ResolveMeshItem(string linkName, string originalUri, string urdfDirectory, string customMeshDirectory, bool isCollision)
        {
            var item = new MeshPathItem
            {
                linkName = linkName,
                originalUri = originalUri,
                cleanFileName = Path.GetFileName(originalUri),
                isCollision = isCollision
            };

            string extension = Path.GetExtension(item.cleanFileName).ToLowerInvariant();
            string baseName = Path.GetFileNameWithoutExtension(item.cleanFileName);
            bool isUnsupportedFormat = UnsupportedModelExtensions.Contains(extension);

            string normUrdfDir = Path.GetFullPath(urdfDirectory).Replace('\\', '/');

            // 1. If original extension is supported (.stl, .dae, .obj, .prefab, .fbx), try resolving directly
            if (!isUnsupportedFormat)
            {
                // A. Check custom mesh directory if user provided one
                if (!string.IsNullOrEmpty(customMeshDirectory) && Directory.Exists(customMeshDirectory))
                {
                    string directInCustom = Path.Combine(customMeshDirectory, item.cleanFileName).Replace('\\', '/');
                    if (File.Exists(directInCustom))
                    {
                        item.status = MeshResolutionStatus.RemappedInProject;
                        item.resolvedAssetPath = GetProjectRelativePath(directInCustom);
                        item.resolvedRelativePath = $"meshes/{item.cleanFileName}";
                        return item;
                    }

                    // Check subfolders (visual / collision / meshes)
                    string[] subfolders = { "meshes", isCollision ? "collision" : "visual", "models" };
                    foreach (var sub in subfolders)
                    {
                        string checkSub = Path.Combine(customMeshDirectory, sub, item.cleanFileName).Replace('\\', '/');
                        if (File.Exists(checkSub))
                        {
                            item.status = MeshResolutionStatus.RemappedInProject;
                            item.resolvedAssetPath = GetProjectRelativePath(checkSub);
                            item.resolvedRelativePath = $"meshes/{item.cleanFileName}";
                            return item;
                        }
                    }

                    // Recursive search in custom mesh folder
                    try
                    {
                        var matchingCustom = Directory.GetFiles(customMeshDirectory, item.cleanFileName, SearchOption.AllDirectories);
                        if (matchingCustom.Length > 0)
                        {
                            item.status = MeshResolutionStatus.RemappedInProject;
                            item.resolvedAssetPath = GetProjectRelativePath(matchingCustom[0].Replace('\\', '/'));
                            item.resolvedRelativePath = $"meshes/{item.cleanFileName}";
                            return item;
                        }
                    }
                    catch { }
                }

                // B. Check local meshes/ subfolder first (ideal self-contained location)
                string inLocalMeshesPath = Path.Combine(urdfDirectory, "meshes", item.cleanFileName).Replace('\\', '/');
                if (File.Exists(inLocalMeshesPath))
                {
                    item.status = MeshResolutionStatus.DirectlyFound;
                    item.resolvedAssetPath = GetProjectRelativePath(inLocalMeshesPath);
                    item.resolvedRelativePath = $"meshes/{item.cleanFileName}";
                    return item;
                }

                // C. Check direct path relative to urdfDirectory
                string directCheckPath = CleanUriPath(originalUri);
                string combinedPath = Path.GetFullPath(Path.Combine(urdfDirectory, directCheckPath)).Replace('\\', '/');

                if (File.Exists(combinedPath))
                {
                    // Check if combinedPath is inside urdfDirectory without traversal (e.g. not ../)
                    if (combinedPath.StartsWith(normUrdfDir + "/", StringComparison.OrdinalIgnoreCase))
                    {
                        item.status = MeshResolutionStatus.DirectlyFound;
                        item.resolvedAssetPath = GetProjectRelativePath(combinedPath);
                        item.resolvedRelativePath = GetRelativePath(urdfDirectory, combinedPath);
                        return item;
                    }
                    else
                    {
                        // File exists outside urdfDirectory (e.g. in ../meshes/ or a parent ROS package folder).
                        // In Unity's official importer, paths starting with "../" are explicitly stripped/broken!
                        // Therefore, mark as RemappedInProject and map to local meshes/{cleanFileName} so it will be copied locally.
                        item.status = MeshResolutionStatus.RemappedInProject;
                        item.resolvedAssetPath = GetProjectRelativePath(combinedPath);
                        item.resolvedRelativePath = $"meshes/{item.cleanFileName}";
                        return item;
                    }
                }

                // D. Check parent/sibling package folders (e.g. ../meshes/ or ../models/)
                string parentDir = Path.GetDirectoryName(urdfDirectory);
                if (!string.IsNullOrEmpty(parentDir))
                {
                    string siblingMeshes = Path.Combine(parentDir, "meshes", item.cleanFileName).Replace('\\', '/');
                    if (File.Exists(siblingMeshes))
                    {
                        item.status = MeshResolutionStatus.RemappedInProject;
                        item.resolvedAssetPath = GetProjectRelativePath(siblingMeshes);
                        item.resolvedRelativePath = $"meshes/{item.cleanFileName}";
                        return item;
                    }
                }

                // E. Check project AssetDatabase for exact cleanFileName
                string foundExact = FindAssetByFileName(item.cleanFileName);
                if (!string.IsNullOrEmpty(foundExact))
                {
                    item.status = MeshResolutionStatus.RemappedInProject;
                    item.resolvedAssetPath = foundExact;
                    item.resolvedRelativePath = $"meshes/{item.cleanFileName}";
                    return item;
                }
            }

            // 2. If unsupported format (like .gltf/.glb) or not found: search project for alternative supported format
            string foundCompatible = FindCompatibleAssetForBaseName(baseName);
            if (!string.IsNullOrEmpty(foundCompatible))
            {
                item.status = MeshResolutionStatus.FormatSubstituted;
                item.resolvedAssetPath = foundCompatible;
                item.resolvedRelativePath = $"meshes/{Path.GetFileName(foundCompatible)}";
                return item;
            }

            // 3. Not found in any supported 3D format: mark for Fallback Primitive replacement
            item.status = MeshResolutionStatus.FallbackPrimitive;
            item.resolvedAssetPath = "";
            item.resolvedRelativePath = "";
            return item;
        }

        private static string CleanUriPath(string uri)
        {
            string path = uri;
            if (path.StartsWith("file://", StringComparison.OrdinalIgnoreCase))
            {
                path = path.Substring(7);
            }
            else if (path.StartsWith("package://", StringComparison.OrdinalIgnoreCase))
            {
                int firstSlash = path.IndexOf('/', 10);
                path = firstSlash > 0 ? path.Substring(firstSlash + 1) : path.Substring(10);
            }
            return path;
        }

        private static string FindAssetByFileName(string fileName)
        {
            string baseName = Path.GetFileNameWithoutExtension(fileName);
            string[] matchingGuids = AssetDatabase.FindAssets(baseName);

            foreach (var guid in matchingGuids)
            {
                string assetPath = AssetDatabase.GUIDToAssetPath(guid);
                if (Path.GetFileName(assetPath).Equals(fileName, StringComparison.OrdinalIgnoreCase))
                {
                    return assetPath;
                }
            }
            return null;
        }

        private static string FindCompatibleAssetForBaseName(string baseName)
        {
            string[] matchingGuids = AssetDatabase.FindAssets(baseName);

            var candidates = new List<string>();
            foreach (var guid in matchingGuids)
            {
                string assetPath = AssetDatabase.GUIDToAssetPath(guid);
                string file = Path.GetFileName(assetPath);
                string ext = Path.GetExtension(file).ToLowerInvariant();

                if (Path.GetFileNameWithoutExtension(file).Equals(baseName, StringComparison.OrdinalIgnoreCase) &&
                    SupportedModelExtensions.Contains(ext))
                {
                    candidates.Add(assetPath);
                }
            }

            if (candidates.Count == 0) return null;

            // Pick highest priority (.prefab -> .stl -> .dae -> .obj -> .fbx)
            var best = candidates.OrderBy(c =>
            {
                string ext = Path.GetExtension(c).ToLowerInvariant();
                if (ext == ".prefab") return 0;
                if (ext == ".stl") return 1;
                if (ext == ".dae") return 2;
                if (ext == ".obj") return 3;
                return 4;
            }).FirstOrDefault();

            return best;
        }

        /// <summary>
        /// Stages a URDF (from a .urdf file, a folder, or a .zip archive, located inside or outside the project),
        /// ensuring all assets reside inside the Unity Assets folder and are 100% loadable
        /// without modal popups or missing file aborts.
        /// </summary>
        public static string StageAndSanitizeUrdf(string inputPath, string customMeshDir = null, string targetProjectFolder = null)
        {
            if (string.IsNullOrEmpty(inputPath))
            {
                throw new ArgumentException("Input URDF or ZIP path cannot be null or empty.");
            }

            string resolvedUrdfPath = inputPath;

            // Handle ZIP Archive (.zip)
            if (inputPath.EndsWith(".zip", StringComparison.OrdinalIgnoreCase))
            {
                string zipName = Path.GetFileNameWithoutExtension(inputPath);
                string extractDir = !string.IsNullOrEmpty(targetProjectFolder)
                    ? targetProjectFolder
                    : Path.Combine(Application.dataPath, "ImportedRovers", zipName);

                if (!Directory.Exists(extractDir)) Directory.CreateDirectory(extractDir);
                ExtractZipArchive(inputPath, extractDir);

                var urdfFiles = Directory.GetFiles(extractDir, "*.urdf", SearchOption.AllDirectories)
                    .Where(f => !f.EndsWith("_sanitized.urdf", StringComparison.OrdinalIgnoreCase))
                    .ToArray();

                if (urdfFiles.Length == 0)
                {
                    throw new FileNotFoundException($"No .urdf file found inside extracted zip '{inputPath}'");
                }
                resolvedUrdfPath = urdfFiles[0];
            }
            else if (Directory.Exists(inputPath))
            {
                var urdfFiles = Directory.GetFiles(inputPath, "*.urdf", SearchOption.AllDirectories)
                    .Where(f => !f.EndsWith("_sanitized.urdf", StringComparison.OrdinalIgnoreCase))
                    .ToArray();

                if (urdfFiles.Length == 0)
                {
                    throw new FileNotFoundException($"No .urdf file found inside directory '{inputPath}'");
                }
                resolvedUrdfPath = urdfFiles[0];
            }

            resolvedUrdfPath = Path.GetFullPath(resolvedUrdfPath);
            if (!File.Exists(resolvedUrdfPath))
            {
                throw new FileNotFoundException($"URDF file does not exist at '{resolvedUrdfPath}'");
            }

            string projectPath = Path.GetFullPath(Application.dataPath).Replace('\\', '/');
            string normalizedUrdf = resolvedUrdfPath.Replace('\\', '/');
            bool isInsideProject = normalizedUrdf.StartsWith(projectPath, StringComparison.OrdinalIgnoreCase);

            string targetUrdfPath;
            if (!isInsideProject)
            {
                string robotBaseName = Path.GetFileNameWithoutExtension(resolvedUrdfPath);
                string targetDir = !string.IsNullOrEmpty(targetProjectFolder)
                    ? targetProjectFolder
                    : Path.Combine(Application.dataPath, "ImportedRovers", robotBaseName);

                if (!Directory.Exists(targetDir)) Directory.CreateDirectory(targetDir);

                string targetMeshes = Path.Combine(targetDir, "meshes");
                if (!Directory.Exists(targetMeshes)) Directory.CreateDirectory(targetMeshes);

                targetUrdfPath = Path.Combine(targetDir, Path.GetFileName(resolvedUrdfPath));
                File.Copy(resolvedUrdfPath, targetUrdfPath, true);
                Debug.Log($"[Rover Importer] Staged external URDF to project: {targetUrdfPath}");
            }
            else
            {
                targetUrdfPath = resolvedUrdfPath;
            }

            return GenerateSanitizedUrdf(targetUrdfPath, null, true, customMeshDir);
        }

        /// <summary>
        /// Rewrites the URDF XML by:
        /// 1. Copying external/sibling meshes into a local meshes/ folder next to the sanitized URDF.
        /// 2. Rewriting all mesh paths to clean "meshes/filename.ext" (eliminating ../ and broken package:// prefixes).
        /// 3. Substituting .gltf/.glb meshes with compatible .STL/.dae/.obj assets found in project.
        /// 4. Replacing missing/unsupported meshes with clean primitive geometries (<cylinder>/<box>).
        /// 5. Auto-healing zero-mass links and zero-inertia tensors so PhysX ArticulationBody creates reliably.
        /// 6. Auto-injecting wheel collision cylinders to prevent falling through terrain.
        /// </summary>
        public static string GenerateSanitizedUrdf(string originalUrdfPath, string outputPath = null, bool syncMeshesToLocal = true, string customMeshDirectory = null)
        {
            if (string.IsNullOrEmpty(outputPath))
            {
                string dir = Path.GetDirectoryName(originalUrdfPath);
                string filename = Path.GetFileNameWithoutExtension(originalUrdfPath);
                outputPath = Path.Combine(dir, $"{filename}_sanitized.urdf");
            }

            var analysis = AnalyzeUrdf(originalUrdfPath, customMeshDirectory);
            XDocument doc = XDocument.Load(originalUrdfPath);
            string urdfDirectory = Path.GetDirectoryName(originalUrdfPath);
            string localMeshesDir = Path.Combine(urdfDirectory, "meshes");

            if (syncMeshesToLocal && !Directory.Exists(localMeshesDir))
            {
                Directory.CreateDirectory(localMeshesDir);
            }

            // 1. Mesh mapping & synchronization: copy all referenced project meshes into local meshes/ folder
            var uriToReplacement = new Dictionary<string, MeshPathItem>();
            foreach (var item in analysis.meshItems)
            {
                if (!uriToReplacement.ContainsKey(item.originalUri))
                {
                    uriToReplacement[item.originalUri] = item;
                }

                if (syncMeshesToLocal && item.status != MeshResolutionStatus.FallbackPrimitive && !string.IsNullOrEmpty(item.resolvedAssetPath))
                {
                    try
                    {
                        string sourceFile = Path.GetFullPath(item.resolvedAssetPath);
                        string destFileName = Path.GetFileName(item.resolvedAssetPath);
                        string destFile = Path.Combine(localMeshesDir, destFileName);

                        // If file is not already inside local meshes/ folder, copy it
                        if (!File.Exists(destFile) && File.Exists(sourceFile))
                        {
                            File.Copy(sourceFile, destFile, true);
                            Debug.Log($"[Rover Importer] Synced mesh: {destFileName} -> {destFile}");
                        }

                        // If .gltf, also sync accompanying .bin buffer file
                        if (sourceFile.EndsWith(".gltf", StringComparison.OrdinalIgnoreCase))
                        {
                            string binSource = Path.ChangeExtension(sourceFile, ".bin");
                            if (File.Exists(binSource))
                            {
                                string binDest = Path.Combine(localMeshesDir, Path.GetFileName(binSource));
                                if (!File.Exists(binDest))
                                {
                                    File.Copy(binSource, binDest, true);
                                    Debug.Log($"[Rover Importer] Synced GLTF buffer: {Path.GetFileName(binSource)}");
                                }
                            }
                        }

                        // Also sync any sibling .asset submesh files (e.g. BaseName_0.asset)
                        string sourceDir = Path.GetDirectoryName(sourceFile);
                        string baseName = Path.GetFileNameWithoutExtension(sourceFile);
                        if (Directory.Exists(sourceDir))
                        {
                            var siblingAssets = Directory.GetFiles(sourceDir, $"{baseName}_*.asset");
                            foreach (var sa in siblingAssets)
                            {
                                string saDest = Path.Combine(localMeshesDir, Path.GetFileName(sa));
                                if (!File.Exists(saDest))
                                {
                                    File.Copy(sa, saDest, true);
                                }
                            }
                        }

                        // Local relative path is guaranteed to be clean meshes/filename (no ../)
                        item.resolvedRelativePath = $"meshes/{destFileName}";
                    }
                    catch (Exception ex)
                    {
                        Debug.LogWarning($"[Rover Importer] Could not copy mesh '{item.resolvedAssetPath}' to local meshes: {ex.Message}");
                    }
                }
            }

            // 1a. Proactively copy all texture files from Textures/ subfolders into local meshes folder
            if (syncMeshesToLocal)
            {
                string[] possibleTextureDirs = {
                    Path.Combine(urdfDirectory, "Textures"),
                    Path.Combine(urdfDirectory, "textures"),
                    Path.Combine(urdfDirectory, "meshes", "Textures"),
                    Path.Combine(urdfDirectory, "meshes", "textures"),
                    Path.Combine(urdfDirectory, "materials", "textures")
                };

                string destTexturesDir = Path.Combine(localMeshesDir, "Textures");
                foreach (var ptd in possibleTextureDirs)
                {
                    if (Directory.Exists(ptd))
                    {
                        if (!Directory.Exists(destTexturesDir)) Directory.CreateDirectory(destTexturesDir);
                        foreach (var imgFile in Directory.GetFiles(ptd))
                        {
                            string ext = Path.GetExtension(imgFile).ToLowerInvariant();
                            if (ext == ".jpg" || ext == ".png" || ext == ".tga" || ext == ".jpeg")
                            {
                                string fName = Path.GetFileName(imgFile);
                                string destImg = Path.Combine(destTexturesDir, fName);
                                if (!File.Exists(destImg)) File.Copy(imgFile, destImg, true);
                                string destImgRoot = Path.Combine(localMeshesDir, fName);
                                if (!File.Exists(destImgRoot)) File.Copy(imgFile, destImgRoot, true);
                            }
                        }
                    }
                }
            }

            // 1b. Texture synchronization: resolve and copy all textures referenced in <texture filename="...">
            if (syncMeshesToLocal)
            {
                var textures = doc.Descendants("texture").ToList();
                foreach (var tex in textures)
                {
                    string originalTexUri = tex.Attribute("filename")?.Value;
                    if (!string.IsNullOrEmpty(originalTexUri))
                    {
                        string cleanTexName = Path.GetFileName(originalTexUri);
                        string resolvedTexPath = ResolveTexturePath(cleanTexName, urdfDirectory, customMeshDirectory);
                        if (!string.IsNullOrEmpty(resolvedTexPath) && File.Exists(resolvedTexPath))
                        {
                            string destTexFile = Path.Combine(localMeshesDir, cleanTexName);
                            if (!File.Exists(destTexFile))
                            {
                                try
                                {
                                    File.Copy(resolvedTexPath, destTexFile, true);
                                    Debug.Log($"[Rover Importer] Synced texture: {cleanTexName} -> {destTexFile}");
                                }
                                catch { }
                            }
                            tex.SetAttributeValue("filename", $"meshes/{cleanTexName}");
                        }
                        else
                        {
                            tex.SetAttributeValue("filename", $"meshes/{cleanTexName}");
                        }
                    }
                }
            }

            // 2. Process links in XML: Meshes, Fallbacks, Physics & Collisions
            var robotElem = doc.Root;
            if (robotElem != null)
            {
                // Validate kinematic tree structure (ensuring valid single-parent spanning tree and connected links)
                SanitizeKinematicTree(robotElem);

                var links = robotElem.Elements("link").ToList();
                foreach (var link in links)
                {
                    string linkName = link.Attribute("name")?.Value ?? "";
                    string lNameLower = linkName.ToLowerInvariant();
                    bool isWheel = analysis.candidateWheelLinks.Contains(linkName) || lNameLower.Contains("wheel") || lNameLower.Contains("tire") || lNameLower.Contains("tyre");
                    bool isChassis = lNameLower.Contains("chassis") || lNameLower.Contains("base_link") || lNameLower.Contains("body_chassis") || lNameLower == "body" || lNameLower == "trunk";
                    bool isSuspension = lNameLower.Contains("rocker") || lNameLower.Contains("bogie") || lNameLower.Contains("diff");
                    bool isSteer = lNameLower.Contains("steer");

                    // A. Update Visual Geometries
                    foreach (var visual in link.Elements("visual").ToList())
                    {
                        var geom = visual.Element("geometry");
                        var mesh = geom?.Element("mesh");
                        if (mesh != null)
                        {
                            string filename = mesh.Attribute("filename")?.Value;
                            if (!string.IsNullOrEmpty(filename) && uriToReplacement.TryGetValue(filename, out var rep))
                            {
                                if (rep.status == MeshResolutionStatus.FallbackPrimitive)
                                {
                                    mesh.Remove();
                                    if (isWheel)
                                    {
                                        geom.Add(new XElement("cylinder", new XAttribute("radius", "0.26"), new XAttribute("length", "0.32")));
                                    }
                                    else if (isChassis)
                                    {
                                        geom.Add(new XElement("box", new XAttribute("size", "1.5 1.0 0.5")));
                                    }
                                    else
                                    {
                                        geom.Add(new XElement("sphere", new XAttribute("radius", "0.08")));
                                    }
                                }
                                else if (!string.IsNullOrEmpty(rep.resolvedRelativePath))
                                {
                                    mesh.SetAttributeValue("filename", rep.resolvedRelativePath.Replace('\\', '/'));
                                }
                            }
                        }
                    }

                    // B. Update Collision Geometries
                    foreach (var collision in link.Elements("collision").ToList())
                    {
                        var geom = collision.Element("geometry");
                        var mesh = geom?.Element("mesh");
                        if (mesh != null)
                        {
                            string filename = mesh.Attribute("filename")?.Value;
                            if (!string.IsNullOrEmpty(filename) && uriToReplacement.TryGetValue(filename, out var rep))
                            {
                                if (rep.status == MeshResolutionStatus.FallbackPrimitive)
                                {
                                    mesh.Remove();
                                    if (isWheel)
                                    {
                                        geom.Add(new XElement("cylinder", new XAttribute("radius", "0.26"), new XAttribute("length", "0.32")));
                                    }
                                    else if (isChassis)
                                    {
                                        geom.Add(new XElement("box", new XAttribute("size", "1.5 1.0 0.5")));
                                    }
                                    else
                                    {
                                        geom.Add(new XElement("sphere", new XAttribute("radius", "0.08")));
                                    }
                                }
                                else if (!string.IsNullOrEmpty(rep.resolvedRelativePath))
                                {
                                    mesh.SetAttributeValue("filename", rep.resolvedRelativePath.Replace('\\', '/'));
                                }
                            }
                        }
                    }

                    // C. Auto-Inject Wheel Collision if completely missing
                    if (isWheel && link.Element("collision") == null)
                    {
                        var col = new XElement("collision",
                            new XElement("origin", new XAttribute("xyz", "0 0 0"), new XAttribute("rpy", "1.57079632679 0 0")),
                            new XElement("geometry",
                                new XElement("cylinder", new XAttribute("radius", "0.26"), new XAttribute("length", "0.32"))
                            )
                        );
                        link.Add(col);
                    }

                    // D. Auto-Heal Mass & Inertia
                    var inertial = link.Element("inertial");
                    if (inertial == null)
                    {
                        inertial = new XElement("inertial");
                        link.Add(inertial);
                    }

                    var massElem = inertial.Element("mass");
                    double massVal = 0;
                    if (massElem != null)
                    {
                        double.TryParse(massElem.Attribute("value")?.Value, NumberStyles.Any, CultureInfo.InvariantCulture, out massVal);
                    }

                    if (massElem == null || massVal <= 0.0001)
                    {
                        double targetMass = 2.0;
                        if (isChassis) targetMass = 250.0;
                        else if (isSuspension) targetMass = 25.0;
                        else if (isWheel) targetMass = 15.0;
                        else if (isSteer) targetMass = 5.0;
                        else if (lNameLower == "ground") targetMass = 1.0;

                        if (massElem == null)
                        {
                            massElem = new XElement("mass");
                            inertial.Add(massElem);
                        }
                        massElem.SetAttributeValue("value", targetMass.ToString("F2", CultureInfo.InvariantCulture));
                        massVal = targetMass;
                    }

                    var inertiaElem = inertial.Element("inertia");
                    double ixx = 0, iyy = 0, izz = 0;
                    if (inertiaElem != null)
                    {
                        double.TryParse(inertiaElem.Attribute("ixx")?.Value, NumberStyles.Any, CultureInfo.InvariantCulture, out ixx);
                        double.TryParse(inertiaElem.Attribute("iyy")?.Value, NumberStyles.Any, CultureInfo.InvariantCulture, out iyy);
                        double.TryParse(inertiaElem.Attribute("izz")?.Value, NumberStyles.Any, CultureInfo.InvariantCulture, out izz);
                    }

                    if (inertiaElem == null || ixx <= 0.00001 || iyy <= 0.00001 || izz <= 0.00001)
                    {
                        if (inertiaElem == null)
                        {
                            inertiaElem = new XElement("inertia");
                            inertial.Add(inertiaElem);
                        }
                        double diag = Math.Max(0.02, massVal * 0.05);
                        if (isChassis) diag = Math.Max(12.0, massVal * 0.1);
                        else if (isWheel) diag = Math.Max(0.3, massVal * 0.02);

                        inertiaElem.SetAttributeValue("ixx", diag.ToString("F4", CultureInfo.InvariantCulture));
                        inertiaElem.SetAttributeValue("ixy", "0.0");
                        inertiaElem.SetAttributeValue("ixz", "0.0");
                        inertiaElem.SetAttributeValue("iyy", diag.ToString("F4", CultureInfo.InvariantCulture));
                        inertiaElem.SetAttributeValue("iyz", "0.0");
                        inertiaElem.SetAttributeValue("izz", diag.ToString("F4", CultureInfo.InvariantCulture));
                    }
                }
            }

            doc.Save(outputPath);
            AssetDatabase.Refresh();

            // Proactively bake any local STL files so Unity has properly-formed prefabs with real meshes ahead of time
            if (Directory.Exists(localMeshesDir))
            {
                var stlFiles = Directory.GetFiles(localMeshesDir, "*.stl", SearchOption.TopDirectoryOnly)
                    .Concat(Directory.GetFiles(localMeshesDir, "*.STL", SearchOption.TopDirectoryOnly))
                    .Distinct();
                foreach (var stl in stlFiles)
                {
                    string assetRel = GetProjectRelativePath(stl);
                    BakeStlPrefab(assetRel);
                }
                AssetDatabase.SaveAssets();
                AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
            }

            Debug.Log($"[Rover Importer] Successfully generated sanitized URDF at: {outputPath}");
            return outputPath;
        }

        private static void SanitizeKinematicTree(XElement robotElem)
        {
            if (robotElem == null) return;

            var links = robotElem.Elements("link").ToList();
            if (links.Count == 0) return;

            // 1. Ensure unique link names
            var seenNames = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var l in links)
            {
                string name = l.Attribute("name")?.Value ?? "unnamed_link";
                if (seenNames.Contains(name))
                {
                    string unique = $"{name}_dup_{Guid.NewGuid().ToString("N").Substring(0, 4)}";
                    l.SetAttributeValue("name", unique);
                    seenNames.Add(unique);
                }
                else
                {
                    seenNames.Add(name);
                }
            }

            var joints = robotElem.Elements("joint").ToList();
            var validLinkNames = new HashSet<string>(links.Select(l => l.Attribute("name")?.Value), StringComparer.OrdinalIgnoreCase);

            // 2. Remove joints with missing parent or child
            var validJoints = new List<XElement>();
            foreach (var j in joints)
            {
                string parent = j.Element("parent")?.Attribute("link")?.Value;
                string child = j.Element("child")?.Attribute("link")?.Value;

                if (!string.IsNullOrEmpty(parent) && !string.IsNullOrEmpty(child) &&
                    validLinkNames.Contains(parent) && validLinkNames.Contains(child) &&
                    !string.Equals(parent, child, StringComparison.OrdinalIgnoreCase))
                {
                    validJoints.Add(j);
                }
                else
                {
                    j.Remove();
                }
            }

            // 3. Find primary root link
            // In a valid tree, the true root has in-degree == 0 (it is NEVER the child of any joint).
            var childLinkSet = new HashSet<string>(validJoints.Select(j => j.Element("child")?.Attribute("link")?.Value), StringComparer.OrdinalIgnoreCase);
            var unparentedLinks = links.Where(l => !childLinkSet.Contains(l.Attribute("name")?.Value)).ToList();

            string primaryRoot;
            if (unparentedLinks.Count == 1)
            {
                // Unambiguous natural root link (e.g. "body" on Spot, "base_link" on Husky)
                primaryRoot = unparentedLinks[0].Attribute("name")?.Value;
            }
            else if (unparentedLinks.Count > 1)
            {
                // Multiple disconnected roots: prioritize standard chassis/body root names, or the root with the most direct children
                var bestCandidate = unparentedLinks
                    .OrderByDescending(l =>
                    {
                        string n = l.Attribute("name")?.Value?.ToLowerInvariant() ?? "";
                        if (n.Contains("chassis") || n.Contains("body") || n.Contains("trunk")) return 100;
                        if (n.Contains("base_link") || n.Contains("base_footprint")) return 90;
                        if (n.Contains("root") || n.Contains("base")) return 80;
                        return 0;
                    })
                    .ThenByDescending(l => validJoints.Count(j => string.Equals(j.Element("parent")?.Attribute("link")?.Value, l.Attribute("name")?.Value, StringComparison.OrdinalIgnoreCase)))
                    .First();
                primaryRoot = bestCandidate.Attribute("name")?.Value;
            }
            else
            {
                // Fallback if cyclic or empty: pick standard chassis link or first link
                var fallbackCandidate = links.FirstOrDefault(l =>
                {
                    string n = l.Attribute("name")?.Value?.ToLowerInvariant() ?? "";
                    return n.Contains("body") || n.Contains("chassis") || n.Contains("base_link") || n.Contains("base_footprint");
                }) ?? links[0];
                primaryRoot = fallbackCandidate.Attribute("name")?.Value;
            }

            // 4. Traverse joint graph starting from primaryRoot to eliminate cycles and multi-parent joints
            var parentToJoints = new Dictionary<string, List<XElement>>(StringComparer.OrdinalIgnoreCase);
            foreach (var j in validJoints)
            {
                string p = j.Element("parent")?.Attribute("link")?.Value;
                if (!parentToJoints.ContainsKey(p)) parentToJoints[p] = new List<XElement>();
                parentToJoints[p].Add(j);
            }

            var visitedLinks = new HashSet<string>(StringComparer.OrdinalIgnoreCase) { primaryRoot };
            var queue = new Queue<string>();
            queue.Enqueue(primaryRoot);

            var activeTreeJoints = new List<XElement>();

            while (queue.Count > 0)
            {
                string curr = queue.Dequeue();
                if (parentToJoints.TryGetValue(curr, out var childrenJoints))
                {
                    foreach (var j in childrenJoints)
                    {
                        string child = j.Element("child")?.Attribute("link")?.Value;
                        if (!visitedLinks.Contains(child))
                        {
                            visitedLinks.Add(child);
                            queue.Enqueue(child);
                            activeTreeJoints.Add(j);
                        }
                        else
                        {
                            // Cycle or multi-parent detected! Remove to preserve strict ArticulationBody tree
                            j.Remove();
                        }
                    }
                }
            }

            // 5. Connect any truly orphaned links to primaryRoot with a fixed joint
            foreach (var l in links)
            {
                string lName = l.Attribute("name")?.Value;
                if (!visitedLinks.Contains(lName))
                {
                    var fixJoint = new XElement("joint",
                        new XAttribute("name", $"joint_auto_connect_{lName}"),
                        new XAttribute("type", "fixed"),
                        new XElement("parent", new XAttribute("link", primaryRoot)),
                        new XElement("child", new XAttribute("link", lName)),
                        new XElement("origin", new XAttribute("xyz", "0 0 0"), new XAttribute("rpy", "0 0 0"))
                    );
                    robotElem.Add(fixJoint);
                    activeTreeJoints.Add(fixJoint);
                    visitedLinks.Add(lName);
                }
            }

            // 6. Ensure the very first joint element under <robot> connects from primaryRoot so Robot.FindRootLink finds primaryRoot in 1 step without looping
            var rootJoint = activeTreeJoints.FirstOrDefault(j => string.Equals(j.Element("parent")?.Attribute("link")?.Value, primaryRoot, StringComparison.OrdinalIgnoreCase));
            var firstJointElem = robotElem.Elements("joint").FirstOrDefault();
            if (rootJoint != null && firstJointElem != null && rootJoint != firstJointElem)
            {
                rootJoint.Remove();
                firstJointElem.AddBeforeSelf(rootJoint);
            }
        }

        private static string ResolveTexturePath(string textureFileName, string urdfDirectory, string customMeshDirectory)
        {
            string clean = Path.GetFileName(textureFileName);

            if (!string.IsNullOrEmpty(customMeshDirectory) && Directory.Exists(customMeshDirectory))
            {
                string p = Path.Combine(customMeshDirectory, clean);
                if (File.Exists(p)) return p;
                string pTex = Path.Combine(customMeshDirectory, "textures", clean);
                if (File.Exists(pTex)) return pTex;
            }

            if (!string.IsNullOrEmpty(urdfDirectory) && Directory.Exists(urdfDirectory))
            {
                string p = Path.Combine(urdfDirectory, clean);
                if (File.Exists(p)) return p;
                string pMesh = Path.Combine(urdfDirectory, "meshes", clean);
                if (File.Exists(pMesh)) return pMesh;
                string pTex = Path.Combine(urdfDirectory, "textures", clean);
                if (File.Exists(pTex)) return pTex;
                string pMat = Path.Combine(urdfDirectory, "materials", "textures", clean);
                if (File.Exists(pMat)) return pMat;

                string parent = Path.GetDirectoryName(urdfDirectory);
                if (!string.IsNullOrEmpty(parent))
                {
                    string pParentTex = Path.Combine(parent, "textures", clean);
                    if (File.Exists(pParentTex)) return pParentTex;
                    string pParentMat = Path.Combine(parent, "materials", "textures", clean);
                    if (File.Exists(pParentMat)) return pParentMat;
                }
            }

            string baseName = Path.GetFileNameWithoutExtension(clean);
            string[] guids = AssetDatabase.FindAssets($"{baseName} t:Texture2D");
            if (guids.Length > 0)
            {
                return AssetDatabase.GUIDToAssetPath(guids[0]);
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

        private static string GetRelativePath(string fromDirectory, string toPath)
        {
            try
            {
                Uri fromUri = new Uri(AppendSlash(fromDirectory));
                Uri toUri = new Uri(toPath);
                Uri relativeUri = fromUri.MakeRelativeUri(toUri);
                return Uri.UnescapeDataString(relativeUri.ToString());
            }
            catch
            {
                return Path.GetFileName(toPath);
            }
        }

        private static string AppendSlash(string path)
        {
            if (!path.EndsWith("/") && !path.EndsWith("\\"))
                return path + "/";
            return path;
        }

        public static void BakeStlPrefab(string stlAssetPath)
        {
            try
            {
                string stlNormalized = stlAssetPath.Replace('\\', '/');
                string fullPath = Path.GetFullPath(stlAssetPath);
                if (!File.Exists(fullPath)) return;

                Mesh[] meshes = StlImporter.ImportMesh(stlNormalized);
                if (meshes == null || meshes.Length == 0) return;

                string dir = Path.GetDirectoryName(stlNormalized).Replace('\\', '/');
                string baseName = Path.GetFileNameWithoutExtension(stlNormalized);
                string prefabPath = $"{dir}/{baseName}.prefab";

                Shader shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");

                // 1. Create and save sub-mesh asset files
                var loadedMeshes = new List<Mesh>();
                for (int i = 0; i < meshes.Length; i++)
                {
                    string meshAssetPath = $"{dir}/{baseName}_{i}.asset";
                    AssetDatabase.CreateAsset(meshes[i], meshAssetPath);
                    loadedMeshes.Add(meshes[i]);
                }
                AssetDatabase.SaveAssets();

                // 2. Remove any corrupt pre-existing prefab
                if (AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath) != null)
                {
                    AssetDatabase.DeleteAsset(prefabPath);
                }

                // 3. Build GameObject hierarchy with assigned sharedMesh and material
                // NOTE: DO NOT use HideFlags.HideAndDontSave, which prevents PrefabUtility from serializing child meshes!
                GameObject parent = new GameObject(baseName);

                for (int i = 0; i < loadedMeshes.Count; i++)
                {
                    GameObject child = new GameObject($"{baseName}_{i}");
                    child.transform.SetParent(parent.transform, false);
                    var mf = child.AddComponent<MeshFilter>();
                    mf.sharedMesh = loadedMeshes[i];
                    var mr = child.AddComponent<MeshRenderer>();
                    var mat = new Material(shader);
                    mat.name = $"{baseName}_mat";
                    mr.sharedMaterial = mat;
                }

                PrefabUtility.SaveAsPrefabAsset(parent, prefabPath);
                UnityEngine.Object.DestroyImmediate(parent);
                AssetDatabase.SaveAssets();
                Debug.Log($"[Rover Importer] Successfully baked STL prefab: {prefabPath} ({loadedMeshes.Count} sub-mesh(es))");
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[Rover Importer] Proactive STL prefab bake failed for '{stlAssetPath}': {ex.Message}");
            }
        }

        public static void ExtractZipArchive(string zipFilePath, string destinationDirectory)
        {
            if (!File.Exists(zipFilePath))
            {
                throw new FileNotFoundException($"Zip file not found: {zipFilePath}");
            }

            if (!Directory.Exists(destinationDirectory))
            {
                Directory.CreateDirectory(destinationDirectory);
            }

            try
            {
                // Attempt standard ZipFile via reflection to ensure zero missing assembly reference warnings across all Unity versions
                var asm = System.Reflection.Assembly.Load("System.IO.Compression.FileSystem");
                var zipFileType = asm?.GetType("System.IO.Compression.ZipFile");
                var extractMethod = zipFileType?.GetMethod("ExtractToDirectory", new Type[] { typeof(string), typeof(string) });
                if (extractMethod != null)
                {
                    extractMethod.Invoke(null, new object[] { zipFilePath, destinationDirectory });
                    AssetDatabase.Refresh();
                    Debug.Log($"[Rover Importer] Successfully extracted ZIP archive to: {destinationDirectory}");
                    return;
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[Rover Importer] ZipFile reflection notice: {ex.Message}. Falling back to system extraction.");
            }

            // Fallback for Windows: powershell Expand-Archive
            try
            {
                var psi = new System.Diagnostics.ProcessStartInfo
                {
                    FileName = "powershell",
                    Arguments = $"-NoProfile -Command \"Expand-Archive -Path '{zipFilePath.Replace("'", "''")}' -DestinationPath '{destinationDirectory.Replace("'", "''")}' -Force\"",
                    CreateNoWindow = true,
                    UseShellExecute = false
                };
                var p = System.Diagnostics.Process.Start(psi);
                p.WaitForExit(30000);
                AssetDatabase.Refresh();
                Debug.Log($"[Rover Importer] Successfully extracted ZIP archive via system to: {destinationDirectory}");
            }
            catch (Exception ex)
            {
                throw new Exception($"Failed to extract zip file '{zipFilePath}': {ex.Message}");
            }
        }
    }
}
