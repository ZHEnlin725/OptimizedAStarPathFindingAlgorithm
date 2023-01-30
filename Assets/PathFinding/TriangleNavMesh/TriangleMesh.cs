using System.Collections.Generic;
using PathFinding.Core;
using PathFinding.TriangleNavMesh.BSP;
using UnityEngine;

namespace PathFinding.TriangleNavMesh
{
    public struct SharedSide : IRoute<Triangle>
    {
        public Vector3 p0, p1;

        public SharedSide(Vector3 p0, Vector3 p1, Triangle origin, Triangle dest)
        {
            this.p0 = p0;
            this.p1 = p1;
            Origin = origin;
            Dest = dest;
        }

        public float Cost => 1;

        public Triangle Origin { get; }

        public Triangle Dest { get; }
    }

    public class TriangleMesh : ITopology<Triangle>
    {
        public int numNodes => _triangles.Count;

        protected BSPTree _bspTree;
        protected Vector3 _planeRotation;
        protected List<Triangle> _triangles;
        protected Dictionary<Triangle, IList<IRoute<Triangle>>> _triangleRoutes;

        public Vector3 planeRotation => _planeRotation;
        
        public virtual void Initialize(int[] indices, Vector3[] vertices, Vector3 planeRotation = default)
        {
            InitTriangles(indices, vertices);

            InitRoutes(indices, vertices);

            InitBSP(planeRotation);
            
            _planeRotation = planeRotation;
        }

        protected virtual void InitTriangles(int[] indices, Vector3[] vertices)
        {
#if UNITY_EDITOR
            var now = System.DateTime.Now;
#endif

            var length = indices.Length;

            #region Init Triangles

            //Init Triangles
            _triangles = new List<Triangle>(length / 3);
            _triangleRoutes = new Dictionary<Triangle, IList<IRoute<Triangle>>>(length * 2 / 3);
            for (int i = 0; i < length;)
                _triangles.Add(new Triangle(vertices[indices[i++]], vertices[indices[i++]], vertices[indices[i++]]));

            #endregion

#if UNITY_EDITOR
            Debug.LogError($"Init Triangles Consume {(System.DateTime.Now - now).TotalMilliseconds}ms ！！！");
#endif
        }

        protected virtual void InitRoutes(int[] indices, Vector3[] vertices)
        {
#if UNITY_EDITOR
            var now = System.DateTime.Now;
#endif
            var length = indices.Length;
            _triangleRoutes = new Dictionary<Triangle, IList<IRoute<Triangle>>>(length * 2 / 3);
            //Init routes
            var indices0 = new int[3];
            var indices1 = new int[3];
            for (int i = 0; i < length;)
            {
                var ti = i / 3;
                indices0[0] = indices[i++];
                indices0[1] = indices[i++];
                indices0[2] = indices[i++];

                for (int j = i; j < length;)
                {
                    var tj = j / 3;
                    indices1[0] = indices[j++];
                    indices1[1] = indices[j++];
                    indices1[2] = indices[j++];
                    if (!Mathematics.HasSharedEdgeIndices(indices0, indices1, out var from, out var to)) continue;
                    var origin = _triangles[ti];
                    var dest = _triangles[tj];

                    var side0 = new SharedSide(vertices[from], vertices[to], origin, dest);
                    var side1 = new SharedSide(vertices[to], vertices[from], dest, origin);

                    if (!_triangleRoutes.TryGetValue(_triangles[tj], out var routes))
                        _triangleRoutes.Add(_triangles[tj], routes = new List<IRoute<Triangle>>(3));
                    routes.Add(side1);

                    if (!_triangleRoutes.TryGetValue(_triangles[ti], out routes))
                        _triangleRoutes.Add(_triangles[ti], routes = new List<IRoute<Triangle>>(3));
                    routes.Add(side0);
                    if (routes.Count == 3)
                    {
                        break;
                    }
                }
            }

            for (int i = 0; i < _triangles.Count; i++)
            {
                if (!_triangleRoutes.TryGetValue(_triangles[i], out var list) || list.Count == 0)
                {
                    Debug.LogError($"triangle index {i} has no neighbor");
                }
            }

#if UNITY_EDITOR
            Debug.LogError($"Init Routes Consume {(System.DateTime.Now - now).TotalMilliseconds}ms ！！！");
#endif
        }

        protected virtual void InitBSP(Vector3 planeRotation = default)
        {
#if UNITY_EDITOR
            var now = System.DateTime.Now;
#endif
            _bspTree = new BSPTree();
            _bspTree.Init(_triangles, planeRotation);

#if UNITY_EDITOR
            Debug.LogError($"Init BSPTree Consume {(System.DateTime.Now - now).TotalMilliseconds}ms ！！！");
#endif
        }

        public IList<IRoute<Triangle>> Routes(Triangle node) =>
            _triangleRoutes.TryGetValue(node, out var routes) ? routes : null;

        public float Estimate(Triangle origin, Triangle dest)
        {
            float dstSqr, minimumDstSqr = float.MaxValue;
            const float half = 0.5f;
            var startV01 = (origin.V0 + origin.V1) * half;
            var startV12 = (origin.V1 + origin.V2) * half;
            var startV20 = (origin.V2 + origin.V0) * half;

            var endV01 = (dest.V0 + dest.V1) * half;
            var endV12 = (dest.V1 + dest.V2) * half;
            var endV20 = (dest.V2 + dest.V0) * half;

            if ((dstSqr = Mathematics.DistanceSqr(startV01, endV01)) < minimumDstSqr)
                minimumDstSqr = dstSqr;
            if ((dstSqr = Mathematics.DistanceSqr(startV01, endV12)) < minimumDstSqr)
                minimumDstSqr = dstSqr;
            if ((dstSqr = Mathematics.DistanceSqr(startV01, endV20)) < minimumDstSqr)
                minimumDstSqr = dstSqr;

            if ((dstSqr = Mathematics.DistanceSqr(startV12, endV01)) < minimumDstSqr)
                minimumDstSqr = dstSqr;
            if ((dstSqr = Mathematics.DistanceSqr(startV12, endV12)) < minimumDstSqr)
                minimumDstSqr = dstSqr;
            if ((dstSqr = Mathematics.DistanceSqr(startV12, endV20)) < minimumDstSqr)
                minimumDstSqr = dstSqr;

            if ((dstSqr = Mathematics.DistanceSqr(startV20, endV01)) < minimumDstSqr)
                minimumDstSqr = dstSqr;
            if ((dstSqr = Mathematics.DistanceSqr(startV20, endV12)) < minimumDstSqr)
                minimumDstSqr = dstSqr;
            if ((dstSqr = Mathematics.DistanceSqr(startV20, endV20)) < minimumDstSqr)
                minimumDstSqr = dstSqr;

            return Mathf.Sqrt(minimumDstSqr);
        }

        public Triangle Index(Vector3 pos)
        {
            if (_bspTree != null)
            {
                var projPos = Mathematics.InverseRotate(_planeRotation).MultiplyPoint(pos).ToVector2XZ();
                var index = _bspTree.Query(projPos);
                if (index >= 0 && index < _triangles.Count)
                    return _triangles[index];
            }

            return null;
        }

        #region Draw Gizmos

#if UNITY_EDITOR
        public void OnDrawGizmos()
        {
            if (_triangles == null) return;
            foreach (var triangle in _triangles)
                DrawTriangle(triangle);
        }

        public void DrawTriangle(int index = 0, bool drawNeighbour = false)
        {
            Gizmos.color = Color.red;
            DrawTriangle(_triangles[index]);
            if (drawNeighbour)
            {
                for (int i = 0; i < _triangleRoutes[_triangles[index]].Count; i++)
                {
                    Gizmos.color = Color.blue;
                    DrawTriangle(_triangleRoutes[_triangles[index]][i].Dest);
                }
            }
        }

        public void DrawTriangle(Triangle triangle)
        {
            Gizmos.DrawLine(triangle.V0, triangle.V1);
            Gizmos.DrawLine(triangle.V1, triangle.V2);
            Gizmos.DrawLine(triangle.V2, triangle.V0);
        }
#endif

        #endregion
    }
}