using System.Collections.Generic;
using UnityEngine;

namespace PathFinding.Core
{
    public interface IPath
    {
        IList<Vector3> Waypoints();
    }

    public interface IPath<T> : IPath
    {
        int Count { get; }

        void Add(T node);

        void Reverse();

        void Clear();
    }
}