using UnityEngine;

#if ROS_PRESENT
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
#endif

public class MapVisualizer : MonoBehaviour
{
    [Header("Wall Settings")]
    public GameObject wallPrefab;
    public float wallHeight = 1.5f;
    public float wallSpawnY = 0.0f; 

#if ROS_PRESENT
    // Internal map state
    private float resolution;
    private int width, height;
    private Vector3 origin;
    private bool initialized = false;

    // Track wall blocks by cell coordinate
    private Dictionary<Vector2Int, GameObject> wallMap = new();

    void Start()
    {
        Debug.Log("[MapVisualizer] Subscribing to /map...");
        ROSConnection.GetOrCreateInstance().Subscribe<OccupancyGridMsg>("/map", OnMapReceived);
        
#if ROS_PRESENT
void Start()
{
    ROSConnection.GetOrCreateInstance().Subscribe<OccupancyGridMsg>("/map", OnMapReceived);
    

}
#endif

    }

    void OnMapReceived(OccupancyGridMsg msg)
    {
        Debug.Log($"[MapVisualizer] Received map: {msg.info.width}x{msg.info.height} @ {msg.info.resolution} m/cell");

        resolution = msg.info.resolution;
        width = (int)msg.info.width;
        height = (int)msg.info.height;

        if (!initialized)
        {
            origin = new Vector3(
                (float)msg.info.origin.position.x,
                0f,
                (float)msg.info.origin.position.y
            );
            initialized = true;
        }

        int occupiedCount = 0;
        int newWalls = 0;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int i = x + y * width;
                sbyte cellValue = msg.data[i];

                if (cellValue != 100)
                    continue;

                occupiedCount++;

                Vector2Int cell = new Vector2Int(x, y);
                if (wallMap.ContainsKey(cell))
                    continue; // Already spawned

                // Compute position centered around origin
                float worldX = x * resolution;
                float worldZ = y * resolution;
                Vector3 mapOffset = new Vector3(-width * resolution / 2f, 0, -height * resolution / 2f);
                Vector3 pos = new Vector3(worldX, wallSpawnY + wallHeight / 2f, worldZ) + mapOffset;

                if (float.IsNaN(pos.x) || float.IsInfinity(pos.x) || float.IsNaN(pos.z))
                {
                    Debug.LogWarning($"[MapVisualizer] Skipped invalid wall at {x},{y} => {pos}");
                    continue;
                }

                GameObject wall = Instantiate(wallPrefab, pos, Quaternion.identity, this.transform);
                wallMap[cell] = wall;
                newWalls++;
            }
        }

        Debug.Log($"[MapVisualizer] Occupied cells: {occupiedCount}, New walls created: {newWalls}");
    }
#endif
}
