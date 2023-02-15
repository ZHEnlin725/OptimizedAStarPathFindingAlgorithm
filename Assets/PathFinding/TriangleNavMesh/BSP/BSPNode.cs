using System;
using System.Collections.Generic;
using UnityEngine;

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
        public Vector2 p0, p1;

        public Segment(Vector2 p0, Vector2 p1)
        {
            this.p0 = p0;
            this.p1 = p1;
        }

        public Vector2 vector => p1 - p0;

        public override string ToString()
        {
            return $"segment [{p0}]=>[{p1}]";
        }
    }

    public class TriangleRef
    {
        public int refIndex;
        public Vector2 v0, v1, v2;

        public TriangleRef(int refIndex, Vector3 v0, Vector3 v1, Vector3 v2)
            : this(refIndex, v0.ToVector2XZ(), v1.ToVector2XZ(), v2.ToVector2XZ())
        {
        }

        public TriangleRef(int refIndex, Vector2 v0, Vector2 v1, Vector2 v2)
        {
            this.refIndex = refIndex;
            (this.v0, this.v1, this.v2) = (v0, v1, v2);
        }

        public Vector2 this[int i]
        {
            get
            {
                Vector2 result;
                switch (i)
                {
                    case 0:
                        result = v0;
                        break;
                    case 1:
                        result = v1;
                        break;
                    case 2:
                        result = v2;
                        break;
                    default:
                        throw new ArgumentOutOfRangeException($"Index {i} Out Of Range !!!");
                }

                return result;
            }
        }

        public bool Intersects(Vector2 point)
        {
            var result = false;
            const int length = 3;
            float x = point.x, y = point.y;
            for (int i = 0, j = length - 1; i < length; j = i++)
                if (this[i].y > y != this[j].y > y &&
                    x < (this[j].x - this[i].x) * (y - this[i].y) / (this[j].y - this[i].y) +
                    this[i].x)
                    result = !result;

            return result;
        }

        public void OnDrawGizmos(Vector3 offset = default)
        {
#if UNITY_EDITOR
            Gizmos.DrawLine(v0.ToVector3XZ(offset), v1.ToVector3XZ(offset));
            Gizmos.DrawLine(v1.ToVector3XZ(offset), v2.ToVector3XZ(offset));
            Gizmos.DrawLine(v2.ToVector3XZ(offset), v0.ToVector3XZ(offset));
#endif
        }
    }

    public class BSPNode
    {
        private const int MIN_TRI_COUNT = 3;

        private const int LEFT_CHILD_INDEX = 0;
        private const int RIGHT_CHILD_INDEX = 1;
        private BSPNode[] _children;

        private int _depth;

        private int _id;
        private readonly List<Vector2> _lVertices = new List<Vector2>(4);

        private readonly List<Vector2> _rVertices = new List<Vector2>(4);

        private Segment _splitline;
        private List<TriangleRef> _triangles;

        public bool IsLeaf => _children == null;

        public BSPNode LeftChild => _children[LEFT_CHILD_INDEX];

        public BSPNode RightChild => _children[RIGHT_CHILD_INDEX];

        public int Query(Vector2 pos)
        {
            var index = -1;

            if (IsLeaf)
            {
                foreach (var triangle in _triangles)
                {
                    if (!triangle.Intersects(pos)) continue;
                    index = triangle.refIndex;
                    break;
                }
            }
            else
            {
                var cross = Mathematics.Cross(_splitline.vector, pos - _splitline.p0);
                if (Math.Abs(cross) < float.Epsilon)
                {
                    index = RightChild.Query(pos);
                    if (index == -1)
                        index = LeftChild.Query(pos);
                }
                else if (cross < 0)
                {
                    index = LeftChild.Query(pos);
                }
                else
                {
                    index = RightChild.Query(pos);
                }
            }

            return index;
        }

        public void Init(BSPTree tree, List<TriangleRef> triangleRefs, int id, int depth = 0)
        {
            _id = id;
            _depth = depth;
            _triangles = triangleRefs;

            if (depth > BSPTree.MaxDepth) return;

            if (triangleRefs.Count <= MIN_TRI_COUNT) return;

            _splitline = SelectSplittingSegment(triangleRefs);
            _children = new[] {new BSPNode(), new BSPNode()};
            var lTris = new List<TriangleRef>();
            var rTris = new List<TriangleRef>();
            foreach (var triangle in triangleRefs)
            {
                var side = SideOfSegment(_splitline, triangle);
                switch (side)
                {
                    case Side.Left:
                        lTris.Add(triangle);
                        break;
                    case Side.Right:
                        rTris.Add(triangle);
                        break;
                    default:
                        SplitTriangle(lTris, rTris, triangle);
                        break;
                }
            }

            LeftChild.Init(tree, lTris, (id << 1) + LEFT_CHILD_INDEX, depth + 1);
            RightChild.Init(tree, rTris, (id << 1) + RIGHT_CHILD_INDEX, depth + 1);
        }

        private void SplitTriangle(List<TriangleRef> left, List<TriangleRef> right,
            TriangleRef triangleRef)
        {
            _rVertices.Clear();
            _lVertices.Clear();

            var numVertices = 3;
            var v0 = triangleRef[numVertices - 1];
            var side_v0 = SideOfSegment(_splitline, v0);
            for (var i = 0; i < numVertices; i++)
            {
                var v1 = triangleRef[i];
                var side_v1 = SideOfSegment(_splitline, v1);
                if (side_v1 == Side.Right)
                {
                    if (side_v0 == Side.Left)
                    {
                        var intersection = Mathematics.Intersection(_splitline.p0, _splitline.p1, v0, v1);
                        _lVertices.Add(intersection);
                        _rVertices.Add(intersection);
                    }
                    else if (side_v0 == Side.Over)
                    {
                        if (_rVertices.Count == 0 || _rVertices[_rVertices.Count - 1] != v0)
                            _rVertices.Add(v0);
                        // Debug.Log("Add rVertex 1");
                    }

                    _rVertices.Add(v1);
                }
                else if (side_v1 == Side.Left)
                {
                    if (side_v0 == Side.Right)
                    {
                        var intersection = Mathematics.Intersection(_splitline.p0, _splitline.p1, v0, v1);
                        _lVertices.Add(intersection);
                        _rVertices.Add(intersection);
                    }
                    else if (side_v0 == Side.Over)
                    {
                        if (_lVertices.Count == 0 || _lVertices[_lVertices.Count - 1] != v0)
                            _lVertices.Add(v0);
                        // Debug.Log("Add lVertex 1");
                    }

                    _lVertices.Add(v1);
                }
                else
                {
                    if (side_v0 == Side.Right)
                    {
                        if (!(_rVertices.Count == 3 && _rVertices[0] == v1))
                            _rVertices.Add(v1);
                        // Debug.Log("Add rVertex 2");
                    }
                    else if (side_v0 == Side.Left)
                    {
                        if (!(_lVertices.Count == 3 && _lVertices[0] == v1))
                            _lVertices.Add(v1);
                        // Debug.Log("Add lVertex 2");
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
                    AddTriangle(right, _rVertices[0], _rVertices[1], _rVertices[2], triangleRef.refIndex);
                }
                else
                {
                    Debug.Assert(right_vertices_count == 4, $"Impossible vertices count {right_vertices_count}");
                    AddTriangle(right, _rVertices[0], _rVertices[1], _rVertices[2], triangleRef.refIndex);
                    AddTriangle(right, _rVertices[0], _rVertices[2], _rVertices[3], triangleRef.refIndex);
                }
            }

            if (left_vertices_count >= 3)
            {
                if (left_vertices_count == 3)
                {
                    AddTriangle(left, _lVertices[0], _lVertices[1], _lVertices[2], triangleRef.refIndex);
                }
                else
                {
                    Debug.Assert(left_vertices_count == 4, $"Impossible vertices count {left_vertices_count}");
                    AddTriangle(left, _lVertices[0], _lVertices[1], _lVertices[2], triangleRef.refIndex);
                    AddTriangle(left, _lVertices[0], _lVertices[2], _lVertices[3], triangleRef.refIndex);
                }
            }
        }

        private void AddTriangle(ICollection<TriangleRef> triangleRefs, Vector2 v0,
            Vector2 v1, Vector2 v2, int index)
        {
            if (v0 == v1 || v1 == v2 || v2 == v0) return;
            triangleRefs.Add(new TriangleRef(index, v0, v1, v2));
        }

        private static Segment SelectSplittingSegment(IReadOnlyCollection<TriangleRef> triangleRefs)
        {
            var minScore = int.MinValue;
            var bestSplittingSegment = new Segment();
            var enumCount = (int) Side.EnumCount;
            var splitCounter = new int[enumCount];
            var count = triangleRefs.Count;
            const int numVertices = 3;
            foreach (var triangle in triangleRefs)
                for (var i = 0; i < numVertices; i++)
                {
                    var segment = new Segment(triangle[i], triangle[(i + 1) % numVertices]);
                    for (var j = 0; j < enumCount; j++)
                        splitCounter[j] = 0;
                    foreach (var tri in triangleRefs)
                        splitCounter[(int) SideOfSegment(segment, tri)]++;
                    var leftCount = splitCounter[(int) Side.Left];
                    var rightCount = splitCounter[(int) Side.Right];
                    var splitCount = splitCounter[(int) Side.Over];
                    if ((leftCount == 0 || rightCount == 0)
                        && leftCount + rightCount == count) continue;
                    var sameVal = Math.Min(leftCount, rightCount);
                    var balanceVal = Math.Abs(leftCount - rightCount);
                    var score = sameVal * 3 - balanceVal - splitCount * 2;
                    if (score > minScore)
                    {
                        minScore = score;
                        bestSplittingSegment = segment;
                    }
                }

            return bestSplittingSegment;
        }

        private static Side SideOfSegment(Segment segment, Vector2 p)
        {
            var val = Mathematics.Cross(segment.vector, p - segment.p0);
            if (val == 0) return Side.Over;
            return val > 0 ? Side.Right : Side.Left;
        }

        private static Side SideOfSegment(Segment segment, TriangleRef triangleRef)
        {
            var vector = segment.vector;
            var side_a = Mathematics.Cross(vector, triangleRef.v0 - segment.p0);
            var side_b = Mathematics.Cross(vector, triangleRef.v1 - segment.p0);
            var side_c = Mathematics.Cross(vector, triangleRef.v2 - segment.p0);
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