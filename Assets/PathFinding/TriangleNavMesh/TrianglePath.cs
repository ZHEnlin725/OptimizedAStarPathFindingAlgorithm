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

        public Vector3 eulerAngles;
        
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
                var matrix = Mathematics.InverseRotate(eulerAngles);
                var invMatrix = matrix.inverse;

                var route = _routes[count - 1];
                _last = new SharedSide(dest, dest, route.Origin, route.Dest);
                count++;
                var side = (SharedSide) _routes[0];
                var funnel = new Funnel(
                    matrix.MultiplyPoint(origin),
                    matrix.MultiplyPoint(side.p0),
                    matrix.MultiplyPoint(side.p1));
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
                    var p0 = matrix.MultiplyPoint(side.p0).ToVector2XZ(); //左
                    var p1 = matrix.MultiplyPoint(side.p1).ToVector2XZ(); //右
                    // //小于0在右边 大于0在左边 等于0重叠
                    var cross_l0 = Mathematics.Cross(funnel.LeftVector, p0 - funnel.Apex);
                    var cross_r0 = Mathematics.Cross(funnel.RightVector, p0 - funnel.Apex);

                    if (cross_l0 <= 0)
                    {
                        if (cross_r0 >= 0)
                        {
                            //漏斗缩小 更换左顶点
                            leftIndex = i;
                            funnel.SetLeft(matrix.MultiplyPoint(side.p0));
                        }
                        else
                        {
                            //当左边界越过右边界 用当前漏斗的右顶点作为新漏斗的顶点来构造新的漏斗
                            var apex = new Vector3(funnel.RightPortal.x, funnel.RightPortalY, funnel.RightPortal.y);
                            //并且将该点加入路径点
                            _buffer.Add(invMatrix.MultiplyPoint(apex));
                            funnel.SetApex(apex);
                            i = leftIndex = rightIndex;
                            if (i < count - 1)
                            {
                                side = SharedSide(i + 1);
                                //从当前的新漏斗开始继续构造漏斗
                                funnel = new Funnel(apex, matrix.MultiplyPoint(side.p0),
                                    matrix.MultiplyPoint(side.p1));
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
                            //漏斗缩小 更换右顶点
                            rightIndex = i;
                            funnel.SetRight(matrix.MultiplyPoint(side.p1));
                        }
                        else
                        {
                            //当右边界越过左边界 用当前漏斗的左顶点作为新漏斗的顶点来构造新的漏斗
                            var apex = new Vector3(funnel.LeftPortal.x, funnel.LeftPortalY, funnel.LeftPortal.y);
                            //并且将该点加入路径点
                            _buffer.Add(invMatrix.MultiplyPoint(apex));
                            funnel.SetApex(apex);
                            i = rightIndex = leftIndex;
                            if (i < count - 1)
                            {
                                //从当前的新漏斗开始继续构造漏斗
                                side = SharedSide(i + 1);
                                funnel = new Funnel(apex, matrix.MultiplyPoint(side.p0),
                                    matrix.MultiplyPoint(side.p1));
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