using System.Collections.Generic;
using System.Threading.Tasks;
using PathFinding;
using PathFinding.TriangleNavMesh;
using UnityEngine;

public class PathFindingTest : MonoBehaviour
{
    public Transform player;
    public TextAsset triangleMeshData;

    private TriangleMeshPathFinding pathFinding;

    public bool async;

    public bool drawMesh, drawMeshPath, drawPathPoints;

    private List<Vector3> pathBuffer = new List<Vector3>();

    // Start is called before the first frame update
    void Start()
    {
        if (triangleMeshData)
        {
            var bytes = triangleMeshData.bytes;
            var meshData = new TriangleMeshData();
            meshData.Deserialize(bytes);
            pathFinding = new TriangleMeshPathFinding(meshData);
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (pathFinding != null && Input.GetMouseButtonDown(0))
        {
            var ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out var hitInfo, 10000, 1 << 0))
            {
                if (async)
                {
                    pathFinding.SearchAsync(player.position, hitInfo.point, list =>
                    {
                        pathBuffer.Clear();
                        pathBuffer.AddRange(list);
                        Mathematics.OptimizePoints(pathBuffer);
                    }, () => { Debug.LogError("Failed to finding path !!!"); });
                }
                else
                {
                    if (pathFinding.Search(player.position,
                        hitInfo.point, out var waypoints))
                    {
                        pathBuffer.Clear();
                        pathBuffer.AddRange(waypoints);
                        Mathematics.OptimizePoints(pathBuffer);
                    }
                    else
                    {
                        Debug.LogError("Failed to finding path !!!");
                    }
                }
            }
        }
    }

    private void OnDrawGizmos()
    {
        if (drawMesh)
        {
            if (pathFinding != null) pathFinding.DrawMesh();
        }

        if (drawMeshPath)
        {
            if (pathFinding != null) pathFinding.DrawPath();
        }

        if (drawPathPoints)
        {
            Gizmos.color = Color.magenta;
            for (int i = 1, j = 0; i < pathBuffer.Count; i++)
            {
                Gizmos.DrawLine(pathBuffer[j] + Vector3.up * 0.1f, pathBuffer[i] + Vector3.up * 0.1f);
                j = i;
            }
        }
    }
}