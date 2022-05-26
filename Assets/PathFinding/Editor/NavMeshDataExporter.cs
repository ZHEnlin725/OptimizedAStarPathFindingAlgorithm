using System.IO;
using System.Text;
using PathFinding.TriangleNavMesh;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace PathFinding.Editor
{
    public static class NavMeshDataExporter
    {
        private const string SavedNavMeshPath = "Assets/NavMeshData";
        private const string SavedTriangleMeshPath = "Assets/TriangleMeshData";

        [MenuItem("PathFinding/Export NavMeshData(.obj)")]
        private static void ExportNavMeshData()
        {
            var scene = SceneManager.GetActiveScene();
            if (!Directory.Exists(SavedNavMeshPath))
                Directory.CreateDirectory(SavedNavMeshPath);
            ExportNavMeshData($"{SavedNavMeshPath}/{scene.name}_navMeshData.obj");
            AssetDatabase.Refresh();
        }

        [MenuItem("PathFinding/Export TriangleMeshData(bin)")]
        private static void ExportTriangleMeshData()
        {
            var scene = SceneManager.GetActiveScene();
            if (!Directory.Exists(SavedTriangleMeshPath))
                Directory.CreateDirectory(SavedTriangleMeshPath);
            ExportTriangleMeshData($"{SavedTriangleMeshPath}/{scene.name}_meshData.bytes");
            AssetDatabase.Refresh();
        }

        private static void ExportNavMeshData(string path)
        {
            var triangulation = UnityEngine.AI.NavMesh.CalculateTriangulation();
            NavMeshUtils.MergeVertices(triangulation.indices, triangulation.vertices, out var mergedIndices,
                out var mergedVertices);

            string dumpTriangulation(Vector3[] vertices, int[] indices)
            {
                var navMeshDataBuilder = new StringBuilder();
                foreach (var vertex in vertices)
                    navMeshDataBuilder.AppendLine($"v {vertex.x} {vertex.y} {vertex.z}");
                var length = indices.Length;
                for (int i = 0; i < length; i += 3)
                    navMeshDataBuilder.AppendLine($"f {indices[i] + 1} {indices[i + 1] + 1} {indices[i + 2] + 1}");
                return navMeshDataBuilder.ToString();
            }

            using (var streamWriter = new StreamWriter(path))
                streamWriter.Write(dumpTriangulation(mergedVertices, mergedIndices));
        }

        private static void ExportTriangleMeshData(string path)
        {
            var triangulation = UnityEngine.AI.NavMesh.CalculateTriangulation();
            NavMeshUtils.MergeVertices(triangulation.indices, triangulation.vertices, out var mergedIndices,
                out var mergedVertices);
            var triangleMeshData = new TriangleMeshData(mergedIndices, mergedVertices);
            var bytes = triangleMeshData.Serialize();
            File.WriteAllBytes(path, bytes);
        }
    }
}