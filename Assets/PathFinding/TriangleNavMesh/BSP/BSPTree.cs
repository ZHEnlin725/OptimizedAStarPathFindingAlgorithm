using System.Collections.Generic;
using UnityEngine;

namespace PathFinding.TriangleNavMesh.BSP
{
    public class BSPTree
    {
        public const int MaxDepth = 20;

        private BSPNode _root;

        private List<TriangleWrap> _triangleWraps;

        public void Init(IList<Triangle> triangles)
        {
            var count = triangles.Count;
            _triangleWraps = new List<TriangleWrap>(count);
            for (int i = 0; i < count; i++)
                _triangleWraps.Add(new TriangleWrap(i, triangles[i].V0, triangles[i].V1, triangles[i].V2));
            _root = new BSPNode();
            _root.Init(this, _triangleWraps, 0);
        }

        public int TriangleIndex(Vector3 pos) => _root.TriangleIndex(new Vector2(pos.x, pos.z));

#if UNITY_EDITOR
        public void OnDrawGizmos(int depth = 0) => _root.OnDrawGizmos(depth);
#endif
    }
}