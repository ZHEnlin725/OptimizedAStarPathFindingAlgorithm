﻿using System.Collections.Generic;
using UnityEngine;

namespace PathFinding.TriangleNavMesh.BSP
{
    public class BSPTree
    {
        public const int MaxDepth = 20;

        private BSPNode _root;

        public void Init(IList<Triangle> triangles, Vector3 eulerAngles = default)
        {
            var count = triangles.Count;
            var triangleRefs = new List<TriangleRef>(count);
            var matrix = Mathematics.InverseRotate(eulerAngles);
            for (var i = 0; i < count; i++)
                triangleRefs.Add(new TriangleRef(i,
                    matrix.MultiplyPoint(triangles[i].v0),
                    matrix.MultiplyPoint(triangles[i].v1),
                    matrix.MultiplyPoint(triangles[i].v2)));
            _root = new BSPNode();
            _root.Init(this, triangleRefs, 1);
        }

        public int Query(Vector2 pos)
        {
            return _root.Query(pos);
        }

#if UNITY_EDITOR
        public void OnDrawGizmos(int depth = 0)
        {
            _root.OnDrawGizmos(depth);
        }
#endif
    }
}