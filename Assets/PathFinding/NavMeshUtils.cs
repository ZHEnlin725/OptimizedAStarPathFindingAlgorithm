using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace PathFinding
{
    public static class NavMeshUtils
    {
        private const float Range = 0.2f;

        private static readonly Dictionary<Vector3, int> VertexDict = new Dictionary<Vector3, int>();

        public static void MergeVertices(int[] indices, Vector3[] vertices, out int[] mergedIndices,
            out Vector3[] mergedVertices)
        {
            mergedIndices = null;
            mergedVertices = null;

            var rawIndices = indices;
            var rawVertices = vertices;

            var tempVertices = new List<Vector3>();

            bool mergeable(Vector3 vertex, float gap, Action<Vector3, int> onmerged = null)
            {
                var merge = false;
                for (int i = 0; i < tempVertices.Count; i++)
                {
                    if (!((vertex - tempVertices[i]).sqrMagnitude <= gap))
                        continue;
                    merge = true;
                    if (onmerged != null)
                        onmerged(tempVertices[i], i);
                    break;
                }

                return merge;
            }

            tempVertices.AddRange(rawVertices.Where(rawVertex => !mergeable(rawVertex, Range)));

            mergedIndices = new int[rawIndices.Length];
            mergedVertices = tempVertices.ToArray();

            for (int i = 0; i < rawIndices.Length; i++)
            {
                var upvalue_i = i;
                var upvalue_indices = mergedIndices;
                if (!mergeable(rawVertices[rawIndices[i]], Range,
                    (v, idx) => upvalue_indices[upvalue_i] = idx))
                    mergedIndices[i] = rawIndices[i];
            }

            AmendmentSameVertex(mergedIndices, mergedVertices);
            Debug.Log($"MergedVertices Count:{mergedVertices.Length} , RawVertices Count:{rawVertices.Length}");
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