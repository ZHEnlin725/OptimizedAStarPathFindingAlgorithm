using System.Collections.Generic;
using UnityEngine;

namespace PathFinding.Core
{
    public interface ITopology<T>
    {
        int numNodes { get; }

        T Index(Vector3 position);

        float Estimate(T from, T to);

        IList<IRoute<T>> Routes(T node);
    }
}