﻿using System;
using System.Collections.Generic;
using UnityEngine;

namespace PathFinding
{
    public static class Mathematics
    {
        public static bool HasSharedEdgeIndices(int[] indices0, int[] indices1)
        {
            var length0 = indices0.Length;
            var length1 = indices1.Length;
            for (var i = 0; i < length0; i++)
            {
                var v00 = indices0[i];
                var v01 = indices0[(i + 1) % length0];
                for (var j = 0; j < length1; j++)
                {
                    var v10 = indices1[j];
                    var v11 = indices1[(j + 1) % length1];
                    if (v00 == v10 && v01 == v11 || v00 == v11 && v01 == v10)
                        return true;
                }
            }

            return false;
        }

        public static bool HasSharedEdgeIndices(int[] indices0, int[] indices1, out int startIndex, out int endIndex)
        {
            var length = indices0.Length;
            startIndex = endIndex = -1;
            for (var i = 0; i < length; i++)
            {
                var v00 = indices0[i];
                var v01 = indices0[(i + 1) % length];
                for (var j = 0; j < length; j++)
                {
                    var v10 = indices1[j];
                    var v11 = indices1[(j + 1) % length];
                    if (v00 == v10 && v01 == v11 || v00 == v11 && v01 == v10)
                    {
                        startIndex = v00;
                        endIndex = v01;
                        return true;
                    }
                }
            }

            return false;
        }

        public static Vector3 CalcNormal(Vector3[] vertices)
        {
            var v1 = vertices[1] - vertices[0];
            var v2 = vertices[vertices.Length - 1] - vertices[0];
            return Vector3.Cross(v1, v2).normalized;
        }

        public static int SideOfPlane(Vector3 normal, Vector3[] vertices, Vector3 point)
        {
            int side;
            if (DistToPlane(normal, vertices, point) > 0) side = 1;
            else if (DistToPlane(normal, vertices, point) < 0) side = -1;
            else side = 0;
            return side;
        }

        public static float DistToPlane(Vector3 normal, Vector3[] vertices, Vector3 point)
        {
            var planeDist = -Vector3.Dot(normal, vertices[0]);
            var dist = Vector3.Dot(point, normal) + planeDist;
            return dist;
        }

        public static float NearestSegmentPointDistanceSqr(ref Vector3 nearest, Vector3 start, Vector3 end,
            Vector3 point)
        {
            nearest = start;
            var abX = end.x - start.x;
            var abY = end.y - start.y;
            var abZ = end.z - start.z;
            var abLen2 = abX * abX + abY * abY + abZ * abZ;
            if (abLen2 > 0)
            {
                // Avoid NaN due to the indeterminate form 0/0
                var t = ((point.x - start.x) * abX + (point.y - start.y) * abY + (point.z - start.z) * abZ) / abLen2;
                var s = Mathf.Clamp01(t);
                nearest.x += abX * s;
                nearest.y += abY * s;
                nearest.z += abZ * s;
            }

            return DistanceSqr(nearest, point);
        }

        public static float DistanceSqr(Vector3 a, Vector3 b)
        {
            var vector3 = new Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
            return (float) (vector3.x * (double) vector3.x +
                            vector3.y * (double) vector3.y +
                            vector3.z * (double) vector3.z);
        }

        public static bool Contains(Vector2[] vertices, Vector2 point)
        {
            var inner = false;
            var length = vertices.Length;
            float x = point.x, y = point.y;
            for (int i = 0, j = length - 1; i < length; j = i++)
                if (vertices[i].y > y != vertices[j].y > y &&
                    x < (vertices[j].x - vertices[i].x) * (y - vertices[i].y) / (vertices[j].y - vertices[i].y) +
                    vertices[i].x)
                    inner = !inner;

            return inner;
        }

        public static float Cross(Vector2 lhs, Vector2 rhs)
        {
            return Cross(lhs.x, lhs.y, rhs.x, rhs.y);
        }

        public static float Cross(float x0, float y0, float x1, float y1)
        {
            return x0 * y1 - y0 * x1;
        }

        public static Vector2 Intersection(Vector2 p00, Vector2 p01, Vector2 p10, Vector2 p11)
        {
            var diff = p10 - p00;
            var d1 = p01 - p00;
            var d2 = p11 - p10;
            var demo = Cross(d1, d2); //det
            if (Mathf.Abs(demo) < float.Epsilon) //parallel
                return p00;
            var t1 = Cross(diff, d2) / demo; // Cross(diff,-d2)
            return p00 + (p01 - p00) * t1;
        }

        public static Matrix4x4 InverseRotate(Vector3 localEulerAngles)
        {
            var quaternion = Quaternion.Euler(localEulerAngles);
            return InverseRotate(quaternion);
        }

        public static Matrix4x4 InverseRotate(Quaternion rotation)
        {
            var axisX = rotation * Vector3.right;
            var axisY = rotation * Vector3.up;
            var axisZ = rotation * Vector3.forward;
            return new Matrix4x4(
                new Vector4(axisX.x, axisY.x, axisZ.x, 0),
                new Vector4(axisX.y, axisY.y, axisZ.y, 0),
                new Vector4(axisX.z, axisY.z, axisZ.z, 0),
                new Vector4(0, 0, 0, 1));
        }

        public static void OptimizePoints(IList<Vector3> points, int start = -1, int end = -1, int bezierSoftness = 5)
        {
            var pointsCount = points?.Count ?? 0;
            if (pointsCount > 2)
            {
                var last = pointsCount - 1;
                start = start > 0 ? Mathf.Clamp(0, last, start) : 0;
                end = end > 0 ? Mathf.Clamp(0, last, end) : last;

                var indexesBuffer = new List<int>();
                var pointsBuffer0 = new List<Vector3>();
                int i0 = start, i1 = start + 1, i2 = start + 2;
                var v0 = points[i0]; //首节点必加
                indexesBuffer.Add(i0);
                while (i2 <= end)
                {
                    var v1 = points[i1];
                    var v2 = points[i2];
                    var d1 = (v1 - v0).normalized;
                    var d2 = (v2 - v0).normalized;
                    var angle = Vector3.Angle(d1, d2);
                    if (Mathf.Abs(angle) < float.Epsilon)
                    {
                        indexesBuffer.Remove(i1);
                        indexesBuffer.Add(i2);
                    }
                    else
                    {
                        indexesBuffer.Remove(i1);
                        indexesBuffer.Add(i1);
                        indexesBuffer.Add(i2);
                    }

                    i1 = i2++;
                    v0 = v1;
                }

                foreach (var i in indexesBuffer)
                    pointsBuffer0.Add(points[i]);

                var count = pointsBuffer0.Count;
                points.Clear();
                if (count > 2)
                {
                    var pointsBuffer1 = new List<Vector3>();
                    pointsBuffer1.Add(pointsBuffer0[0]);
                    for (int i = 0; i < count - 2; i++)
                    {
                        Vector3 p0 = pointsBuffer0[i], p1 = pointsBuffer0[i + 1], p2 = pointsBuffer0[i + 2];
                        var p0p1 = Vector3.Lerp(p0, p1, 0.85f);
                        var p2p1 = Vector3.Lerp(p2, p1, 0.85f);
                        for (int j = 0; j <= bezierSoftness; j++)
                            pointsBuffer1.Add(Bezier(p0p1, p1, p2p1, j * 1f / bezierSoftness));
                    }

                    pointsBuffer1.Add(pointsBuffer0[count - 1]);

                    foreach (var point in pointsBuffer1)
                        points.Add(point);
                    // ListPool<Vector3>.Release(pointsBuffer1);
                }
                else
                {
                    foreach (var point in pointsBuffer0)
                        points.Add(point);
                }

                // ListPool<int>.Release(indexesBuffer);
                // ListPool<Vector3>.Release(pointsBuffer0);
            }
        }

        private static Vector3 Bezier(Vector3 p0, Vector3 p1, Vector3 p2, float t)
        {
            var p0p1 = Vector3.Lerp(p0, p1, t);
            var p1p2 = Vector3.Lerp(p1, p2, t);
            return Vector3.Lerp(p0p1, p1p2, t);
        }

        public static Vector2 ToVector2XZ(this Vector3 vector3, Vector2 offset = default)
        {
            return new Vector2(vector3.x + offset.x, vector3.z + offset.y);
        }

        public static Vector3 ToVector3XZ(this Vector2 vector2, Vector3 offset = default)
        {
            return new Vector3(vector2.x + offset.x, offset.y, vector2.y + offset.z);
        }

        public struct Matrix
        {
            internal float M11, M12, M13;
            internal float M21, M22, M23;
            internal float M31, M32, M33;

            public static readonly Matrix Identity;

            public static readonly Matrix Zero;

            static Matrix()
            {
                Zero = new Matrix();

                Identity = new Matrix {M11 = 1, M22 = 1, M33 = 1};
            }

            public Matrix(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32,
                float m33)
            {
                M11 = m11;
                M12 = m12;
                M13 = m13;
                M21 = m21;
                M22 = m22;
                M23 = m23;
                M31 = m31;
                M32 = m32;
                M33 = m33;
            }

            public static Matrix operator *(Matrix value1, Matrix value2)
            {
                Matrix result;
                Multiply(ref value1, ref value2, out result);
                return result;
            }

            public static Matrix operator +(Matrix value1, Matrix value2)
            {
                Matrix result;
                Add(ref value1, ref value2, out result);
                return result;
            }

            public static bool operator ==(Matrix value1, Matrix value2)
            {
                return Math.Abs(value1.M11 - value2.M11) < float.Epsilon &&
                       Math.Abs(value1.M12 - value2.M12) < float.Epsilon &&
                       Math.Abs(value1.M13 - value2.M13) < float.Epsilon &&
                       Math.Abs(value1.M21 - value2.M21) < float.Epsilon &&
                       Math.Abs(value1.M22 - value2.M22) < float.Epsilon &&
                       Math.Abs(value1.M23 - value2.M23) < float.Epsilon &&
                       Math.Abs(value1.M31 - value2.M31) < float.Epsilon &&
                       Math.Abs(value1.M32 - value2.M32) < float.Epsilon &&
                       Math.Abs(value1.M33 - value2.M33) < float.Epsilon;
            }

            public static bool operator !=(Matrix value1, Matrix value2)
            {
                return value1.M11 != value2.M11 ||
                       value1.M12 != value2.M12 ||
                       value1.M13 != value2.M13 ||
                       value1.M21 != value2.M21 ||
                       value1.M22 != value2.M22 ||
                       value1.M23 != value2.M23 ||
                       value1.M31 != value2.M31 ||
                       value1.M32 != value2.M32 ||
                       value1.M33 != value2.M33;
            }

            public static Vector3 operator *(Matrix lhs, Vector3 vector)
            {
                Vector3 result;
                result.x = lhs.M11 * vector.x + lhs.M12 * vector.y + lhs.M13 * vector.z;
                result.y = lhs.M21 * vector.x + lhs.M22 * vector.y + lhs.M23 * vector.z;
                result.z = lhs.M31 * vector.x + lhs.M32 * vector.y + lhs.M33 * vector.z;
                return result;
            }

            public override bool Equals(object obj)
            {
                if (!(obj is Matrix)) return false;
                var other = (Matrix) obj;

                return M11 == other.M11 &&
                       M12 == other.M12 &&
                       M13 == other.M13 &&
                       M21 == other.M21 &&
                       M22 == other.M22 &&
                       M23 == other.M23 &&
                       M31 == other.M31 &&
                       M32 == other.M32 &&
                       M33 == other.M33;
            }

            public override int GetHashCode()
            {
                return M11.GetHashCode() ^
                       M12.GetHashCode() ^
                       M13.GetHashCode() ^
                       M21.GetHashCode() ^
                       M22.GetHashCode() ^
                       M23.GetHashCode() ^
                       M31.GetHashCode() ^
                       M32.GetHashCode() ^
                       M33.GetHashCode();
            }

            public static void Multiply(ref Matrix matrix1, ref Matrix matrix2, out Matrix result)
            {
                var num0 = matrix1.M11 * matrix2.M11 + matrix1.M12 * matrix2.M21 + matrix1.M13 * matrix2.M31;
                var num1 = matrix1.M11 * matrix2.M12 + matrix1.M12 * matrix2.M22 + matrix1.M13 * matrix2.M32;
                var num2 = matrix1.M11 * matrix2.M13 + matrix1.M12 * matrix2.M23 + matrix1.M13 * matrix2.M33;
                var num3 = matrix1.M21 * matrix2.M11 + matrix1.M22 * matrix2.M21 + matrix1.M23 * matrix2.M31;
                var num4 = matrix1.M21 * matrix2.M12 + matrix1.M22 * matrix2.M22 + matrix1.M23 * matrix2.M32;
                var num5 = matrix1.M21 * matrix2.M13 + matrix1.M22 * matrix2.M23 + matrix1.M23 * matrix2.M33;
                var num6 = matrix1.M31 * matrix2.M11 + matrix1.M32 * matrix2.M21 + matrix1.M33 * matrix2.M31;
                var num7 = matrix1.M31 * matrix2.M12 + matrix1.M32 * matrix2.M22 + matrix1.M33 * matrix2.M32;
                var num8 = matrix1.M31 * matrix2.M13 + matrix1.M32 * matrix2.M23 + matrix1.M33 * matrix2.M33;

                result.M11 = num0;
                result.M12 = num1;
                result.M13 = num2;
                result.M21 = num3;
                result.M22 = num4;
                result.M23 = num5;
                result.M31 = num6;
                result.M32 = num7;
                result.M33 = num8;
            }

            public static void Add(ref Matrix matrix1, ref Matrix matrix2, out Matrix result)
            {
                result.M11 = matrix1.M11 + matrix2.M11;
                result.M12 = matrix1.M12 + matrix2.M12;
                result.M13 = matrix1.M13 + matrix2.M13;
                result.M21 = matrix1.M21 + matrix2.M21;
                result.M22 = matrix1.M22 + matrix2.M22;
                result.M23 = matrix1.M23 + matrix2.M23;
                result.M31 = matrix1.M31 + matrix2.M31;
                result.M32 = matrix1.M32 + matrix2.M32;
                result.M33 = matrix1.M33 + matrix2.M33;
            }

            public static void Sub(ref Matrix matrix1, ref Matrix matrix2, out Matrix result)
            {
                result.M11 = matrix1.M11 - matrix2.M11;
                result.M12 = matrix1.M12 - matrix2.M12;
                result.M13 = matrix1.M13 - matrix2.M13;
                result.M21 = matrix1.M21 - matrix2.M21;
                result.M22 = matrix1.M22 - matrix2.M22;
                result.M23 = matrix1.M23 - matrix2.M23;
                result.M31 = matrix1.M31 - matrix2.M31;
                result.M32 = matrix1.M32 - matrix2.M32;
                result.M33 = matrix1.M33 - matrix2.M33;
            }

            public static void Multiply(ref Matrix matrix1, float scaleFactor, out Matrix result)
            {
                var num = scaleFactor;
                result.M11 = matrix1.M11 * num;
                result.M12 = matrix1.M12 * num;
                result.M13 = matrix1.M13 * num;
                result.M21 = matrix1.M21 * num;
                result.M22 = matrix1.M22 * num;
                result.M23 = matrix1.M23 * num;
                result.M31 = matrix1.M31 * num;
                result.M32 = matrix1.M32 * num;
                result.M33 = matrix1.M33 * num;
            }

            public float this[int index]
            {
                get
                {
                    switch (index)
                    {
                        case 0:
                            return M11;
                        case 1:
                            return M12;
                        case 2:
                            return M13;
                        case 3:
                            return M21;
                        case 4:
                            return M22;
                        case 5:
                            return M23;
                        case 6:
                            return M31;
                        case 7:
                            return M32;
                        case 8:
                            return M33;
                        default:
                            throw new IndexOutOfRangeException("Invalid matrix uid!");
                    }
                }
                set
                {
                    switch (index)
                    {
                        case 0:
                            M11 = value;
                            break;
                        case 1:
                            M12 = value;
                            break;
                        case 2:
                            M13 = value;
                            break;
                        case 3:
                            M21 = value;
                            break;
                        case 4:
                            M22 = value;
                            break;
                        case 5:
                            M23 = value;
                            break;
                        case 6:
                            M31 = value;
                            break;
                        case 7:
                            M32 = value;
                            break;
                        case 8:
                            M33 = value;
                            break;
                        default:
                            throw new IndexOutOfRangeException("Invalid matrix uid!");
                    }
                }
            }

            public float this[int row, int col]
            {
                get
                {
                    if (row > 2 || row < 0) throw new IndexOutOfRangeException("Invalid matrix uid!");
                    if (col > 2 || col < 0) throw new IndexOutOfRangeException("Invalid matrix uid!");
                    return this[row * 3 + col];
                }

                set
                {
                    if (row > 2 || row < 0) throw new IndexOutOfRangeException("Invalid matrix uid!");
                    if (col > 2 || col < 0) throw new IndexOutOfRangeException("Invalid matrix uid!");
                    this[row * 3 + col] = value;
                }
            }

            public override string ToString()
            {
                return string.Format("{0}|{1}|{2}\n{3}|{4}|{5}\n{6}|{7}|{8}", M11, M12, M13, M21, M22, M23, M31, M32,
                    M33);
            }
        }
    }
}