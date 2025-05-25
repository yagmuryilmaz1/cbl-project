using UnityEngine;
using System.Collections.Generic;

#if ROS_PRESENT
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
#endif

public class MapVisualizer2 : MonoBehaviour
{
    public GameObject wallPrefab;
    public float wallHeight = 1.5f;

#if ROS_PRESENT
    private Dictionary<Vector2Int, GameObject> wallMap = new();

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<OccupancyGridMsg>("/map", OnMapReceived);

        // === FAKE MAP TEST ===
        var fake = new OccupancyGridMsg();
        fake.info = new MapMetaDataMsg();
        fake.info.width = 10;
        fake.info.height = 10;
        fake.info.resolution = 0.2f;
        fake.info.origin = new PoseMsg(
            new PointMsg(0, 0, 0),
            new QuaternionMsg(0, 0, 0, 1)
        );

        fake.data = new sbyte[fake.info.width * fake.info.height];
        fake.data[22] = 100;
        fake.data[33] = 100;
        fake.data[44] = 100;

        int occupied = 0;
        foreach (var val in fake.data)
            if (val == 100) occupied++;

        Debug.Log("FAKE MAP: Created with " + occupied + " occupied cells");

        OnMapReceived(fake);
    }

    void OnMapReceived(OccupancyGridMsg msg)
    {
        int width = (int)msg.info.width;
        int height = (int)msg.info.height;
        float resolution = msg.info.resolution;

        int occupiedCount = 0;
        int newWalls = 0;

        Vector3 mapOffset = new Vector3(-width * resolution / 2f, 0, -height * resolution / 2f);

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int index = x + y * width;
                if (msg.data[index] != 100)
                    continue;

                occupiedCount++;

                Vector2Int cell = new Vector2Int(x, y);
                if (wallMap.ContainsKey(cell))
                    continue;

                Vector3 pos = new Vector3(x * resolution, wallHeight / 2f, y * resolution) + mapOffset;

                GameObject wall = Instantiate(wallPrefab, pos, Quaternion.identity, transform);
                wallMap[cell] = wall;
                newWalls++;
            }
        }

        Debug.Log($"[MapVisualizer] Occupied cells: {occupiedCount}, New walls created: {newWalls}");
    }
#endif
}
