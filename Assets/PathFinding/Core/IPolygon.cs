using UnityEngine;

namespace PathFinding.Core
{
    public interface IPolygon
    {
        Vector3 this[int index] { get; }
    }
}