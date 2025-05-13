using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class MapFromMaterial : MonoBehaviour
{
    public Texture2D mapTexture;          // <-- assign your JPEG file here
    public GameObject wallPrefab;
    public float resolution = 0.05f;        // Meters per pixel
    public Vector2 origin = Vector2.zero;
    public float wallHeight = 1.5f;

    void Start()
    {
        if (mapTexture == null)
        {
            Debug.LogError("No texture assigned!");
            return;
        }

        Generate3DMap();
    }

    void Generate3DMap()
    {
        int width = mapTexture.width;
        int height = mapTexture.height;

        float mapWidthWorld = width * resolution;
        float mapHeightWorld = height * resolution;
        Vector3 mapOffset = new Vector3(-mapWidthWorld / 2f, 0, -mapHeightWorld / 2f);

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                Color pixel = mapTexture.GetPixel(x, y);
                float brightness = (pixel.r + pixel.g + pixel.b) / 3f;

                if (brightness < 0.4f)
                {
                    float worldX = x * resolution + origin.x;
                    float worldZ = y * resolution + origin.y;
                    Vector3 position = new Vector3(worldX, wallHeight / 2f, worldZ) + mapOffset;

                    Instantiate(wallPrefab, position, Quaternion.identity, transform);
                }
            }
        }
    }
}
