using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using PathFinding.Core;
using UnityEngine;

namespace PathFinding.TriangleNavMesh
{
    public class TriangleMeshPathFinding
    {
        private class TrianglePathFinder : PathFinderBase<Triangle>
        {
        }

        private TrianglePath trianglePath;
        private TriangleMesh triangleMesh;
        private TrianglePathFinder pathfinder;

        public TriangleMeshPathFinding(TriangleMeshData meshData) :
            this(meshData.indices, meshData.vertices)
        {
        }

        public TriangleMeshPathFinding(int[] indices, Vector3[] vertices)
        {
            pathfinder = new TrianglePathFinder();
            trianglePath = new TrianglePath();
            triangleMesh = new TriangleMesh();
            // NavMeshUtils.AmendmentSameVertex(indices, vertices);
            triangleMesh.Initialize(indices, vertices);
        }

        public bool Search(Vector3 from, Vector3 to, out IList<Vector3> waypoints)
        {
            waypoints = null;
            if (pathfinder.Search(triangleMesh, from, to, trianglePath))
            {
                trianglePath.origin = from;
                trianglePath.dest = to;
                waypoints = trianglePath.Waypoints();
                return true;
            }

            return false;
        }

        public async void SearchAsync(Vector3 from, Vector3 to, Action<IList<Vector3>> succeedCallback,
            Action failedCallback)
        {
            using (var searchTask = Task.Run(() => pathfinder.Search(triangleMesh, @from, to, trianglePath)))
            {
                if (await searchTask)
                {
                    trianglePath.origin = from;
                    trianglePath.dest = to;
                    succeedCallback?.Invoke(trianglePath.Waypoints());
                }
                else
                {
                    failedCallback?.Invoke();
                }
            }
        }

        public void DrawPath()
        {
            trianglePath.OnDrawGizmos();
        }

        public void DrawMesh()
        {
            triangleMesh.OnDrawGizmos();
        }
    }
}