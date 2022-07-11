using System;
using System.Collections.Generic;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace PathFinding.TriangleNavMesh.BSP
{
    public enum Side
    {
        Left,
        Right,
        Over,
        EnumCount
    }

    public struct Segment
    {
        private Vector2 _p0, _p1;

        public Segment(Vector2 p0, Vector2 p1)
        {
            _p0 = p0;
            _p1 = p1;
        }

        public Vector2 P0 => _p0;
        public Vector2 P1 => _p1;
        public Vector2 Vector => _p1 - _p0;

        public override string ToString()
        {
            return $"segment [{P0}]=>[{P1}]";
        }
    }

    public class TriangleWrap
    {
        public int Index => _index;

        public Segment[] Borders => _borders;

        public Vector2[] Vertices => _vertices;

        public Vector2 V0 => _vertices[0];
        public Vector2 V1 => _vertices[1];
        public Vector2 V2 => _vertices[2];

        private int _index;
        private Segment[] _borders;
        private Vector2[] _vertices;

        public TriangleWrap(int index, Vector3 v0, Vector3 v1, Vector3 v2) : this(index, v0.ToVector2XZ(),
            v1.ToVector2XZ(), v2.ToVector2XZ())
        {
        }

        public TriangleWrap(int index, Vector2 v0, Vector2 v1, Vector2 v2)
        {
            _index = index;
            _vertices = new[] {v0, v1, v2};
            _borders = new[] {new Segment(v0, v1), new Segment(v1, v2), new Segment(v2, v0)};
        }

        public bool Intersects(Vector2 p) => Mathematics.Contains(_vertices, p);

        public void OnDrawGizmos(Vector3 offset = default)
        {
            Gizmos.DrawLine(V0.ToVector3XZ(offset), V1.ToVector3XZ(offset));
            Gizmos.DrawLine(V1.ToVector3XZ(offset), V2.ToVector3XZ(offset));
            Gizmos.DrawLine(V2.ToVector3XZ(offset), V0.ToVector3XZ(offset));
        }
    }

    public class BSPNode
    {
        private const int MIN_TRI_COUNT = 3;

        private const int LEFT_CHILD_INDEX = 0;
        private const int RIGHT_CHILD_INDEX = 1;

        private int _id;

        private int _depth;

        private Segment _splitLine;
        private BSPNode[] _children;
        private List<TriangleWrap> _triangles;

        public bool IsLeaf => _children == null;

        public BSPNode LeftChild => _children[LEFT_CHILD_INDEX];

        public BSPNode RightChild => _children[RIGHT_CHILD_INDEX];

        public int TriangleIndex(Vector2 pos)
        {
            var index = -1;

            if (IsLeaf)
            {
                foreach (var triangle in _triangles)
                {
                    if (!triangle.Intersects(pos)) continue;
                    index = triangle.Index;
                    break;
                }
            }
            else
            {
                var cross = Mathematics.Cross(_splitLine.Vector, pos - _splitLine.P0);
                if (Math.Abs(cross) < float.Epsilon)
                {
                    index = RightChild.TriangleIndex(pos);
                    if (index == -1)
                        index = LeftChild.TriangleIndex(pos);
                }
                else if (cross < 0)
                {
                    index = LeftChild.TriangleIndex(pos);
                }
                else
                {
                    index = RightChild.TriangleIndex(pos);
                }
            }

            return index;
        }

        public void Init(BSPTree tree, List<TriangleWrap> triangleWraps, int id, int depth = 0)
        {
            _id = id;
            _depth = depth;
            _triangles = triangleWraps;

            if (depth > BSPTree.MaxDepth) return;

            if (triangleWraps.Count <= MIN_TRI_COUNT) return;
            
            _splitLine = SelectSplittingSegment(triangleWraps);
            _children = new[] {new BSPNode(), new BSPNode()};
            var lTris = new List<TriangleWrap>();
            var rTris = new List<TriangleWrap>();
            foreach (var wrap in triangleWraps)
            {
                var side = SplitTriangle(_splitLine, wrap);
                switch (side)
                {
                    case Side.Left:
                        lTris.Add(wrap);
                        break;
                    case Side.Right:
                        rTris.Add(wrap);
                        break;
                    default:
                        SplitTriangle(lTris, rTris, wrap);
                        break;
                }
            }

            LeftChild.Init(tree, lTris, (1 << depth) + LEFT_CHILD_INDEX, depth + 1);
            RightChild.Init(tree, rTris, (1 << depth) + RIGHT_CHILD_INDEX, depth + 1);
        }

        private List<Vector2> _rVertices = new List<Vector2>();
        private List<Vector2> _lVertices = new List<Vector2>();

        private void SplitTriangle(List<TriangleWrap> left, List<TriangleWrap> right,
            TriangleWrap triangle)
        {
            _rVertices.Clear();
            _lVertices.Clear();

            var numVertices = 3;
            var v0 = triangle.Vertices[numVertices - 1];
            var side_v0 = SplitPoint(_splitLine, v0);
            for (int i = 0; i < numVertices; i++)
            {
                var v1 = triangle.Vertices[i];
                var side_v1 = SplitPoint(_splitLine, v1);
                if (side_v1 == Side.Right)
                {
                    if (side_v0 == Side.Left)
                    {
                        var intersection = Mathematics.Intersection(_splitLine.P0, _splitLine.P1, v0, v1);
                        _lVertices.Add(intersection);
                        _rVertices.Add(intersection);
                    }
                    else if (side_v0 == Side.Over)
                    {
                        if (_rVertices.Count == 0 || _rVertices[_rVertices.Count - 1] != v0)
                        {
                            _rVertices.Add(v0);
                            // Debug.Log("Add rVertex 1");
                        }
                    }

                    _rVertices.Add(v1);
                }
                else if (side_v1 == Side.Left)
                {
                    if (side_v0 == Side.Right)
                    {
                        var intersection = Mathematics.Intersection(_splitLine.P0, _splitLine.P1, v0, v1);
                        _lVertices.Add(intersection);
                        _rVertices.Add(intersection);
                    }
                    else if (side_v0 == Side.Over)
                    {
                        if (_lVertices.Count == 0 || _lVertices[_lVertices.Count - 1] != v0)
                        {
                            _lVertices.Add(v0);
                            // Debug.Log("Add lVertex 1");
                        }
                    }

                    _lVertices.Add(v1);
                }
                else
                {
                    if (side_v0 == Side.Right)
                    {
                        if (!(_rVertices.Count == 3 && _rVertices[0] == v1))
                        {
                            _rVertices.Add(v1);
                            // Debug.Log("Add rVertex 2");
                        }
                    }
                    else if (side_v0 == Side.Left)
                    {
                        if (!(_lVertices.Count == 3 && _lVertices[0] == v1))
                        {
                            _lVertices.Add(v1);
                            // Debug.Log("Add lVertex 2");
                        }
                    }
                }

                v0 = v1;
                side_v0 = side_v1;
            }

            var left_vertices_count = _lVertices.Count;
            var right_vertices_count = _rVertices.Count;

            Debug.Assert(right_vertices_count >= 3 || left_vertices_count >= 3);

            if (right_vertices_count >= 3)
            {
                if (right_vertices_count == 3)
                {
                    AddTriangle(right, _rVertices[0], _rVertices[1], _rVertices[2], triangle.Index);
                }
                else
                {
                    Debug.Assert(right_vertices_count == 4, $"Impossible vertices count {right_vertices_count}");
                    AddTriangle(right, _rVertices[0], _rVertices[1], _rVertices[2], triangle.Index);
                    AddTriangle(right, _rVertices[0], _rVertices[2], _rVertices[3], triangle.Index);
                }
            }

            if (left_vertices_count >= 3)
            {
                if (left_vertices_count == 3)
                {
                    AddTriangle(left, _lVertices[0], _lVertices[1], _lVertices[2], triangle.Index);
                }
                else
                {
                    Debug.Assert(left_vertices_count == 4, $"Impossible vertices count {left_vertices_count}");
                    AddTriangle(left, _lVertices[0], _lVertices[1], _lVertices[2], triangle.Index);
                    AddTriangle(left, _lVertices[0], _lVertices[2], _lVertices[3], triangle.Index);
                }
            }
        }

        private void AddTriangle(ICollection<TriangleWrap> triangleWraps, Vector2 v0,
            Vector2 v1, Vector2 v2, int index)
        {
            if (v0 == v1 || v1 == v2 || v2 == v0) return;
            triangleWraps.Add(new TriangleWrap(index, v0, v1, v2));
        }

        private static Segment SelectSplittingSegment(List<TriangleWrap> triangleWraps)
        {
            var minScore = int.MinValue;
            var bestSplittingSegment = new Segment();
            var enumCount = (int) Side.EnumCount;
            var splitCounter = new int[enumCount];
            var count = triangleWraps.Count;
            foreach (var triangleWrap in triangleWraps)
            {
                foreach (var segment in triangleWrap.Borders)
                {
                    for (int i = 0; i < enumCount; i++) splitCounter[i] = 0;

                    foreach (var wrap in triangleWraps)
                        splitCounter[(int) SplitTriangle(segment, wrap)]++;

                    var leftCount = splitCounter[(int) Side.Left];
                    var rightCount = splitCounter[(int) Side.Right];
                    var splitCount = splitCounter[(int) Side.Over];
                    if ((leftCount == 0 || rightCount == 0)
                        && leftCount + rightCount == count) continue;
                    var balanceVal = Math.Abs(leftCount - rightCount);
                    var sameVal = Math.Min(leftCount, rightCount);
                    var score = sameVal * 3 - balanceVal - splitCount * 2;

                    if (score > minScore)
                    {
                        minScore = score;
                        bestSplittingSegment = segment;
                    }
                }
            }

            return bestSplittingSegment;
        }

        private static Side SplitPoint(Segment segment, Vector2 p)
        {
            var val = Mathematics.Cross(segment.Vector, p - segment.P0);
            if (val == 0) return Side.Over;
            return val > 0 ? Side.Right : Side.Left;
        }

        private static Side SplitTriangle(Segment segment, TriangleWrap triangle)
        {
            var vector = segment.Vector;
            var side_a = Mathematics.Cross(vector, triangle.V0 - segment.P0);
            var side_b = Mathematics.Cross(vector, triangle.V1 - segment.P0);
            var side_c = Mathematics.Cross(vector, triangle.V2 - segment.P0);
            var right_side = false;
            if (side_a != 0) right_side = side_a > 0;
            if (side_b != 0) right_side = side_b > 0;
            if (side_c != 0) right_side = side_c > 0;

            if (side_a <= 0 == side_b <= 0 && side_b <= 0 == side_c <= 0)
                return right_side ? Side.Right : Side.Left;

            if (side_a >= 0 == side_b >= 0 && side_b >= 0 == side_c >= 0)
                return right_side ? Side.Right : Side.Left;
            return Side.Over;
        }

        public void OnDrawGizmos(int depth = 0, bool drawGizmos = true)
        {
            if (drawGizmos)
                DrawTriangles();

            if (!IsLeaf)
                foreach (var child in _children)
                    child.OnDrawGizmos(depth, LeftChild == child && depth == child._depth);
        }

        private void DrawTriangles()
        {
            foreach (var triangle in _triangles)
                triangle.OnDrawGizmos(Vector3.up * 0.05f);
        }
    }
}