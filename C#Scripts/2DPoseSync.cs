using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Tf2;
using System;

public class ROSTFSubscriber : MonoBehaviour
{
    [SerializeField]
    private GameObject robotModel;  // This should be assigned in Inspector

    [SerializeField] 
    private string tfTopic = "/tf";

    private ROSConnection ros;

    void Start()
    {
        // Checking if robotModel is assigned
        if (robotModel == null)
        {
            Debug.LogError("Robot model is not assigned!");
            return;
        }

        // Get ROS connection
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to TF topic
        ros.Subscribe<TFMessageMsg>(tfTopic, ProcessTFMessage);

        Debug.Log("Subscribed to " + tfTopic);
    }

    void ProcessTFMessage(TFMessageMsg tfMessage)
    {
        try
        {
            foreach(var transform in tfMessage.transforms)
            {
                if(transform.header.frame_id == "map" && transform.child_frame_id == "base_link")
                {
                    // Convert ROS position to Unity position
                    Vector3 position = new Vector3(
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z
                    );

                    // Convert ROS quaternion to Unity rotation
                    // Note: ROS is right-handed, Unity is left-handed
                    Quaternion rotation = new Quaternion(
                        transform.transform.rotation.x,
                        -transform.transform.rotation.y,  // Flip Y for coordinate system difference
                        -transform.transform.rotation.z,  // Flip Z for coordinate system difference
                        transform.transform.rotation.w
                    );
                    robotModel.transform.position = position;
                    robotModel.transform.rotation = rotation;

                    //Debug info
                    Debug.Log($"Updated robot position: {position}, rotation: {rotation}");
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error processing TF message: {e.Message}");
        }
    }
}
