using PathFinding.Core;
using UnityEngine;

namespace PathFinding.TriangleNavMesh
{
    public class Triangle : IPolygon
    {
        public Vector3 V0
        {
            get => Vertices[0];
            set => Vertices[0] = value;
        }

        public Vector3 V1
        {
            get => Vertices[1];
            set => Vertices[1] = value;
        }

        public Vector3 V2
        {
            get => Vertices[2];
            set => Vertices[2] = value;
        }

        public Vector3 Normal { get; }

        public Vector3[] Vertices { get; }

        public Triangle(Vector3 v0, Vector3 v1, Vector3 v2)
        {
            Vertices = new[] {v0, v1, v2};
            Normal = Mathematics.CalcNormal(Vertices);
        }
    }
}