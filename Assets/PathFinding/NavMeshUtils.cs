using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace PathFinding
{
    public static class NavMeshUtils
    {
        private const float Gap = 0.05f;

        private static readonly Dictionary<Vector3, int> VertexDict = new Dictionary<Vector3, int>();

        public static void MergeVertices(IList<int> indices, IList<Vector3> vertices, out int[] mergedIndices,
            out Vector3[] mergedVertices)
        {
            mergedIndices = null;
            mergedVertices = null;

            var rawIndices = indices;
            var rawVertices = vertices;

            var tempIndices = new List<int>();
            var tempVertices = new List<Vector3>();

            bool needMerge(Vector3 lhs, Vector3 rhs, float gap) =>
                (lhs - rhs).sqrMagnitude <= gap;

            bool mergeable(Vector3 vertex, float gap, Action<Vector3, int> onmerged = null)
            {
                var merge = false;
                for (int i = 0; i < tempVertices.Count; i++)
                {
                    if (!needMerge(vertex, tempVertices[i], gap))
                        continue;
                    merge = true;
                    if (onmerged != null)
                        onmerged(tempVertices[i], i);
                    break;
                }

                return merge;
            }

            tempVertices.AddRange(rawVertices.Where(rawVertex => !mergeable(rawVertex, Gap)));

            mergedVertices = tempVertices.ToArray();

            const float epsilon = 0.001f;

            for (int i = 0; i < rawIndices.Count;)
            {
                var i0 = rawIndices[i++];
                var i1 = rawIndices[i++];
                var i2 = rawIndices[i++];

                var v0 = rawVertices[i0];
                var v1 = rawVertices[i1];
                var v2 = rawVertices[i2];

                if (needMerge(v0, v1, epsilon)
                    || needMerge(v1, v2, epsilon)
                    || needMerge(v2, v0, epsilon))
                    continue;

                tempIndices.Add(i0);
                tempIndices.Add(i1);
                tempIndices.Add(i2);
            }

            var indicesCount = tempIndices.Count;
            mergedIndices = new int[indicesCount];
            for (int i = 0; i < indicesCount; i++)
            {
                var upvalue_i = i;
                var upvalue_indices = mergedIndices;
                if (!mergeable(rawVertices[tempIndices[i]], Gap,
                    (v, idx) => upvalue_indices[upvalue_i] = idx))
                    mergedIndices[i] = tempIndices[i];
            }

            AmendmentSameVertex(mergedIndices, mergedVertices);
            Debug.Log($"RawIndices Count:{rawIndices.Count} => MergedIndices Count:{mergedIndices.Length}");
            Debug.Log($"RawVertices Count:{rawVertices.Count} => MergedVertices Count:{mergedVertices.Length}");
        }

        public static void AmendmentSameVertex(int[] indices, Vector3[] vertices)
        {
            if (indices == null || vertices == null) return;
            VertexDict.Clear();
            var mergedCount = 0;
            for (var i = 0; i < indices.Length; i++)
            {
                if (!VertexDict.TryGetValue(vertices[indices[i]], out var indicesIndex))
                {
                    VertexDict.Add(vertices[indices[i]], i);
                }

                else if (indices[i] != indices[indicesIndex])
                {
                    mergedCount++;
                    indices[i] = indices[indicesIndex];
                }
            }

            Debug.Log($"Indices Merged Count:{mergedCount}");
        }

        public static void ScaleVertex(Vector3[] vertices, Vector2 origin, float scale)
        {
            if (vertices == null || Math.Abs(scale - 1) < float.Epsilon) return;

            for (int i = 0; i < vertices.Length; i++)
            {
                vertices[i].x += -origin.x; // 缩放移动
                vertices[i].z += -origin.y;
                vertices[i] = vertices[i] * scale;
            }
        }
    }
}