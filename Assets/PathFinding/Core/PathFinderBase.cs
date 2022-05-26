using System;
using System.Collections.Generic;
using System.Threading;
using PathFinding.Optimized;
using UnityEngine;

namespace PathFinding.Core
{
    /*
     * astar path finding algorithm
     * author:ZHEnlin
     */
    public abstract class PathFinderBase<T>
    {
        protected enum NodeCategory : byte
        {
            Unvisited,
            Opened,
            Closed
        }

        protected sealed class Node : IComparable<Node>
        {
            public int version;

            public NodeCategory category;

            public T node;
            public IRoute<T> route;

            public float cost;
            public float estimate;

            public float totalCost => cost + estimate;

            public int CompareTo(Node other) => (int) Math.Ceiling(totalCost - other.totalCost);
        }

        private int signals;

        private int version;

        private readonly BinaryHeap<Node> openList;
        private readonly Dictionary<T, Node> nodeDict;

        protected PathFinderBase(int capacity = 32)
        {
            openList = new FastBinaryHeap<Node>(BinaryHeap<Node>.HeapType.Minimum, capacity);
            nodeDict = new Dictionary<T, Node>(capacity);
        }

        public bool Search(ITopology<T> topology, Vector3 from, Vector3 to, IPath<T> path)
        {
            T origin = topology.Index(from), dest = topology.Index(to);

            if (origin == null || dest == null)
            {
                Debug.LogError($"Failed to index origin:{from} or dest:{to}");
                return false;
            }

            if (search(topology, origin, dest))
            {
                path.Clear();

                var node = getOrCreateNode(dest);

                while (!node.node.Equals(origin))
                {
                    path.Add(node.node);

                    node = getOrCreateNode(node.route.Origin);
                }

                path.Reverse();

                return true;
            }

            return false;
        }

        public bool Search(ITopology<T> topology, Vector3 from, Vector3 to, IPath<IRoute<T>> path)
        {
            T origin = topology.Index(from), dest = topology.Index(to);

            if (origin == null || dest == null)
            {
                Debug.LogError($"Failed to index origin:{from} or dest:{to}");
                return false;
            }

            if (search(topology, origin, dest))
            {
                path.Clear();

                var node = getOrCreateNode(dest);

                while (!node.node.Equals(origin))
                {
                    path.Add(node.route);

                    node = getOrCreateNode(node.route.Origin);
                }

                path.Reverse();

                return true;
            }

            return false;
        }

        public void Reset()
        {
            version = 0;
            foreach (var pair in nodeDict)
                pair.Value.version = -1;
        }

        #region Protected Methods

        protected virtual bool search(ITopology<T> topology, T origin, T dest)
        {
            if (signals > 1) return false;

            Interlocked.Increment(ref signals);

            initSearch(topology, origin, dest);

            var found = false;

            do
            {
                var node = openList.Pop();

                node.category = NodeCategory.Closed;

                if (node.node.Equals(dest))
                {
                    found = true;
                    break;
                }

                visitRoutes(topology, node, dest);
                
            } while (openList.Count > 0);

            Interlocked.Decrement(ref signals);

            return found;
        }

        protected virtual void visitRoutes(ITopology<T> topology, Node node, T dest)
        {
            var routes = topology.Routes(node.node);

            var count = routes?.Count ?? -1;

            for (int i = 0; i < count; i++)
            {
                var route = routes[i];
                
                var destNode = getOrCreateNode(route.Dest);

                var cost = node.cost + route.Cost;

                if (destNode.category == NodeCategory.Opened)
                {
                    if (cost > destNode.cost) continue;

                    openList.Remove(destNode);
                }
                else if (destNode.category == NodeCategory.Closed)
                {
                    if (cost > destNode.cost) continue;
                }

                destNode.cost = cost;
                destNode.estimate = topology.Estimate(node.node, dest);
                destNode.route = route;

                pushOpenList(destNode);
            }
        }

        protected virtual void initSearch(ITopology<T> topology, T origin, T dest)
        {
            ++version;
            openList.Clear();
            var node = getOrCreateNode(origin);
            node.cost = 0;
            node.estimate = topology.Estimate(origin, dest);
            pushOpenList(node);
        }

        protected virtual Node getOrCreateNode(T node)
        {
            if (!nodeDict.TryGetValue(node, out var wrap))
                nodeDict.Add(node, wrap = new Node {node = node});
            if (wrap.version != version)
            {
                wrap.route = null;
                wrap.category = NodeCategory.Unvisited;
            }

            wrap.version = version;
            return wrap;
        }

        protected virtual void pushOpenList(Node node)
        {
            openList.Push(node);
            node.category = NodeCategory.Opened;
        }

        #endregion
    }
}