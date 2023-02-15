using System;
using PathFinding.Core;
using UnityEngine;

namespace PathFinding.TriangleNavMesh
{
    public class Triangle : IPolygon
    {
        public Triangle(Vector3 v0, Vector3 v1, Vector3 v2)
        {
            this.v0 = v0;
            this.v1 = v1;
            this.v2 = v2;
        }

        public Vector3 v0 { get; }

        public Vector3 v1 { get; }

        public Vector3 v2 { get; }

        public int numVertices => 3;

        public Vector3 this[int index]
        {
            get
            {
                Vector3 result;
                switch (index)
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
                        throw new ArgumentOutOfRangeException($"Index {index} Out Of Range !!!");
                }

                return result;
            }
        }
    }
}