using System;
using PathFinding.Core;
using UnityEngine;

namespace PathFinding.TriangleNavMesh
{
    public class Triangle : IPolygon
    {
        public Vector3 v0 => _v0;

        public Vector3 v1 => _v1;

        public Vector3 v2 => _v2;

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

        private Vector3 _v0, _v1, _v2;

        public Triangle(Vector3 v0, Vector3 v1, Vector3 v2)
        {
            _v0 = v0;
            _v1 = v1;
            _v2 = v2;
        }
    }
}