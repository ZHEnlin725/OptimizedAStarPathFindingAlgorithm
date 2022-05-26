using System;
using System.Collections.Generic;
using PathFinding.Core;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace PathFinding.TriangleNavMesh
{
    public class TrianglePath : IPath<IRoute<Triangle>>
    {
        private struct Funnel
        {
            public float LeftPortalY, RightPortalY;

            public Vector2 Apex, LeftPortal, RightPortal;

            public Vector2 LeftVector => LeftPortal - Apex;

            public Vector2 RightVector => RightPortal - Apex;

            public Funnel(Vector3 apex, Vector3 leftPortal, Vector3 rightPortal)
            {
                Apex = new Vector2(apex.x, apex.z);
                LeftPortalY = leftPortal.y;
                LeftPortal = new Vector2(leftPortal.x, leftPortal.z);
                RightPortalY = rightPortal.y;
                RightPortal = new Vector2(rightPortal.x, rightPortal.z);
            }

            public void SetApex(Vector3 apex) => Apex = new Vector2(apex.x, apex.z);

            public void SetLeft(Vector3 portal)
            {
                LeftPortalY = portal.y;
                LeftPortal = new Vector2(portal.x, portal.z);
            }

            public void SetRight(Vector3 portal)
            {
                RightPortalY = portal.y;
                RightPortal = new Vector2(portal.x, portal.z);
            }
        }

        public Vector3 origin, dest;

        private SharedSide _last;

        private List<Vector3> _buffer = new List<Vector3>();

        private List<IRoute<Triangle>> _routes = new List<IRoute<Triangle>>();

        public IList<Vector3> Waypoints()
        {
            _buffer.Clear();
            _buffer.Add(origin);
            var count = _routes.Count;
            if (count > 0)
            {
                var route = _routes[count - 1];
                _last = new SharedSide(dest, dest, route.Origin, route.Dest);
                count++;
                var side = (SharedSide) _routes[0];
                var funnel = new Funnel(origin, side.p0, side.p1);
                int leftIndex = 0, rightIndex = 0;
                var time = DateTime.Now;
                for (int i = 1; i < count; i++)
                {
                    if ((DateTime.Now - time).TotalMilliseconds > 1000)
                    {
                        Debug.LogError("Timeout");
                        break;
                    }

                    side = SharedSide(i);
                    var p0 = ToVector2(side.p0); //左
                    var p1 = ToVector2(side.p1); //右
                    // //小于0在右边 大于0在左边 等于0重叠
                    var cross_l0 = Mathematics.Cross(funnel.LeftVector, p0 - funnel.Apex);
                    var cross_r0 = Mathematics.Cross(funnel.RightVector, p0 - funnel.Apex);

                    if (cross_l0 <= 0)
                    {
                        if (cross_r0 >= 0)
                        {
                            leftIndex = i;
                            funnel.SetLeft(side.p0);
                        }
                        else
                        {
                            var apex = new Vector3(funnel.RightPortal.x, funnel.RightPortalY, funnel.RightPortal.y);
                            _buffer.Add(apex);
                            funnel.SetApex(apex);
                            i = leftIndex = rightIndex;
                            if (i < count - 1)
                            {
                                side = SharedSide(i + 1);
                                funnel = new Funnel(apex, side.p0, side.p1);
                                continue;
                            }

                            break;
                        }
                    }

                    var cross_l1 = Mathematics.Cross(funnel.LeftVector, p1 - funnel.Apex);
                    var cross_r1 = Mathematics.Cross(funnel.RightVector, p1 - funnel.Apex);

                    if (cross_r1 >= 0)
                    {
                        if (cross_l1 <= 0)
                        {
                            rightIndex = i;
                            funnel.SetRight(side.p1);
                        }
                        else
                        {
                            var apex = new Vector3(funnel.LeftPortal.x, funnel.LeftPortalY, funnel.LeftPortal.y);
                            _buffer.Add(apex);
                            funnel.SetApex(apex);
                            i = rightIndex = leftIndex;
                            if (i < count - 1)
                            {
                                side = SharedSide(i + 1);
                                funnel = new Funnel(apex, side.p0, side.p1);
                            }
                        }
                    }
                }
            }

            _buffer.Add(dest);
            return _buffer;
        }

        public int Count => _routes.Count;

        public void Add(IRoute<Triangle> node) => _routes.Add(node);

        public void Reverse() => _routes.Reverse();

        public void Clear() => _routes.Clear();

        private SharedSide SharedSide(int index) => (SharedSide) (index < Count ? _routes[index] : _last);

        private static Vector2 ToVector2(Vector3 vector3) => new Vector2(vector3.x, vector3.z);

        #region Draw Gizmos

        public void OnDrawGizmos()
        {
            foreach (SharedSide side in _routes)
            {
                DrawTriangle(side.Origin);
                DrawTriangle(side.Dest);
                var p0 = side.p0 + Vector3.up * 0.1f;
                var p1 = side.p1 + Vector3.up * 0.1f;
                var v = p1 - p0;
                Gizmos.color = Color.red;
                Gizmos.DrawLine(p0, p0 + v * 0.5f);
                Gizmos.color = Color.black;
                Gizmos.DrawLine(p0 + v * 0.5f, p1);
            }
        }

        public void DrawTriangle(Triangle triangle)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(triangle.V0, triangle.V1);
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(triangle.V1, triangle.V2);
            Gizmos.color = Color.green;
            Gizmos.DrawLine(triangle.V2, triangle.V0);
        }

        #endregion
    }
}