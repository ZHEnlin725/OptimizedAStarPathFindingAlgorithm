using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using PathFinding.Core;
using UnityEngine;

namespace PathFinding.TriangleNavMesh
{
    public class TriangleMeshPathFinding
    {
        private readonly TrianglePathFinder pathfinder;
        private readonly TriangleMesh triangleMesh;

        private readonly TrianglePath trianglePath;

        public TriangleMeshPathFinding(TriangleMeshData meshData, Vector3 eulerAngles = default) :
            this(meshData.indices, meshData.vertices, eulerAngles)
        {
        }

        public TriangleMeshPathFinding(int[] indices, Vector3[] vertices, Vector3 eulerAngles = default)
        {
            triangleMesh = new TriangleMesh();
            // NavMeshUtils.AmendmentSameVertex(indices, vertices);
            triangleMesh.Initialize(indices, vertices, eulerAngles);
            trianglePath = new TrianglePath {eulerAngles = eulerAngles};
            pathfinder = new TrianglePathFinder();
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
            using (var searchTask = Task.Run(() => pathfinder.Search(triangleMesh, from, to, trianglePath)))
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

        private class TrianglePathFinder : PathFinderBase<Triangle>
        {
        }
    }
}