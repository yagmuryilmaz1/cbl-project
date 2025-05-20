using UnityEngine;

#if ROS_PRESENT
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Nav;
using System.Collections.Generic;
#endif

public class IncrementalMapVisualizer : MonoBehaviour
{
    public GameObject wallPrefab;
    public float wallHeight = 1.5f;

#if ROS_PRESENT
    private float resolution;
    private int width, height;
    private Vector3 origin;
    private bool isFirstMessage = true;

    // Track cells already visualized
    private Dictionary<Vector2Int, GameObject> wallMap = new();

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<OccupancyGridMsg>("/map", OnMapReceived);
    }

    void OnMapReceived(OccupancyGridMsg msg)
    {
        resolution = msg.info.resolution;
        width = (int)msg.info.width;
        height = (int)msg.info.height;

        if (isFirstMessage)
        {
            origin = new Vector3(
                (float)msg.info.origin.position.x,
                0f,
                (float)msg.info.origin.position.y
            );
            isFirstMessage = false;
        }

        sbyte[] data = msg.data;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int i = x + y * width;
                sbyte value = data[i];

                if (value != 100)
                    continue;

                Vector2Int cellKey = new Vector2Int(x, y);
                if (wallMap.ContainsKey(cellKey))
                    continue; // Already visualized

                Vector3 pos = new Vector3(
                    x * resolution,
                    wallHeight / 2f,
                    y * resolution
                ) + origin;

                if (float.IsNaN(pos.x) || float.IsInfinity(pos.x))
                    continue;

                GameObject wall = Instantiate(wallPrefab, pos, Quaternion.identity, this.transform);
                wallMap[cellKey] = wall;
            }
        }
    }
#endif
}
