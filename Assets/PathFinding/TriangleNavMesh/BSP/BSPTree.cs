using System.Collections.Generic;
using UnityEngine;

namespace PathFinding.TriangleNavMesh.BSP
{
    public class BSPTree
    {
        public const int MaxDepth = 20;

        private BSPNode _root;
        private ProjPlane _projPlane;

        public void Init(IList<Triangle> triangles, ProjPlane projPlane = ProjPlane.XZ)
        {
            var count = triangles.Count;
            var triangleWraps = new List<ProjTriangle>(count);
            for (int i = 0; i < count; i++)
                triangleWraps.Add(new ProjTriangle(i,
                    Mathematics.TransformPointTo(projPlane, triangles[i].V0),
                    Mathematics.TransformPointTo(projPlane, triangles[i].V1),
                    Mathematics.TransformPointTo(projPlane, triangles[i].V2)));
            _root = new BSPNode();
            _root.Init(this, triangleWraps, 0);
            _projPlane = projPlane;
        }

        public int TriangleIndex(Vector3 pos) => _root.TriangleIndex(Mathematics.TransformPointTo(_projPlane, pos).ToVector2XZ());

#if UNITY_EDITOR
        public void OnDrawGizmos(int depth = 0) => _root.OnDrawGizmos(depth);
#endif
    }
}