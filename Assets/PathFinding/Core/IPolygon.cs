using UnityEngine;

namespace PathFinding.Core
{
    public interface IPolygon
    {
        int numVertices { get; }
        
        Vector3 this[int index] { get; }
    }
}