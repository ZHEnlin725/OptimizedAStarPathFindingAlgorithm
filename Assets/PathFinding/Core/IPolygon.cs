using UnityEngine;

namespace PathFinding.Core
{
    public interface IPolygon
    {
        Vector3 Normal { get; }

        Vector3[] Vertices { get; }
    }
}