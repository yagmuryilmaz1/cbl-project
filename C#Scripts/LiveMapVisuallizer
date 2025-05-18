using UnityEngine;

#if ROS_PRESENT
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Nav;
using System.Collections.Generic;
#endif

public class LiveMapVisualizer : MonoBehaviour
{
    public GameObject wallPrefab;
    public float wallHeight = 1.5f;

#if ROS_PRESENT
    private float resolution;
    private int width, height;
    private Vector3 origin;
    private List<GameObject> spawnedWalls = new List<GameObject>();
    private sbyte[] previousData = null;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<OccupancyGridMsg>("/map", OnMapReceived);
    }

    void OnMapReceived(OccupancyGridMsg msg)
    {
        // Skip if map hasn't changed
        if (previousData != null && msg.data.Length == previousData.Length)
        {
            bool isSame = true;
            for (int i = 0; i < msg.data.Length; i++)
            {
                if (msg.data[i] != previousData[i])
                {
                    isSame = false;
                    break;
                }
            }
            if (isSame) return;
        }

        previousData = msg.data;

        // Clear old walls
        foreach (GameObject wall in spawnedWalls)
        {
            Destroy(wall);
        }
        spawnedWalls.Clear();

        resolution = msg.info.resolution;
        width = (int)msg.info.width;
        height = (int)msg.info.height;
        origin = new Vector3(
            (float)msg.info.origin.position.x,
            0f,
            (float)msg.info.origin.position.y
        );

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int i = x + y * width;
                sbyte cell = msg.data[i];

                if (cell == 100)
                {
                    Vector3 pos = new Vector3(
                        x * resolution,
                        wallHeight / 2f,
                        y * resolution
                    ) + origin;

                    GameObject wall = Instantiate(wallPrefab, pos, Quaternion.identity, this.transform);
                    spawnedWalls.Add(wall);
                }
            }
        }

        Debug.Log($"Map updated: {spawnedWalls.Count} walls generated.");
    }
#endif
}
