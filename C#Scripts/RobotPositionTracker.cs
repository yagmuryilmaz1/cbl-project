using UnityEngine;
#if ROS_PRESENT
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
#endif

public class RobotPositionTracker : MonoBehaviour
{
    [Header("Robot References")]
    public Transform robotBaseLink; // Will reference base_link
    public bool smoothMovement = true;
    public float smoothSpeed = 5f;

    [Header("Debug Info")]
    public bool showDebugInfo = true;
    public Vector3 currentPosition;
    public Vector3 currentRotation;

#if ROS_PRESENT
    private Vector3 targetPosition;
    private Quaternion targetRotation;
    private bool hasInitialPose = false;

    void Start()
    {
        // Find base_link automatically if not assigned
        if (robotBaseLink == null)
        {
            robotBaseLink = transform.Find("base_link");
            if (robotBaseLink == null)
            {
                Debug.LogError("[RobotTracker] Could not find base_link! Please assign manually.");
                return;
            }
        }

        Debug.Log("[RobotTracker] Subscribing to /odom...");
        ROSConnection.GetOrCreateInstance().Subscribe<OdometryMsg>("/odom", OnOdometryReceived);

        // Initialize target to current position
        targetPosition = robotBaseLink.position;
        targetRotation = robotBaseLink.rotation;
    }

    void OnOdometryReceived(OdometryMsg odom)
    {
        if (showDebugInfo)
        {
            Debug.Log($"[RobotTracker] Received odom: pos({odom.pose.pose.position.x:F2}, {odom.pose.pose.position.y:F2})");
        }

        // Convert ROS coordinates to Unity coordinates
        // Note: ROS uses X-forward, Z-up; Unity uses Z-forward, Y-up
        targetPosition = new Vector3(
            (float)odom.pose.pose.position.x,
            0f, // Keep robot on ground plane
            (float)odom.pose.pose.position.y
        );

        // Convert quaternion (ROS to Unity coordinate system)
        targetRotation = new Quaternion(
            (float)odom.pose.pose.orientation.x,
            (float)odom.pose.pose.orientation.z, // Swap Y and Z
            (float)odom.pose.pose.orientation.y,
            (float)odom.pose.pose.orientation.w
        );

        hasInitialPose = true;

        if (!smoothMovement && robotBaseLink != null)
        {
            robotBaseLink.position = targetPosition;
            robotBaseLink.rotation = targetRotation;
        }
    }

    void Update()
    {
        if (hasInitialPose && smoothMovement && robotBaseLink != null)
        {
            robotBaseLink.position = Vector3.Lerp(robotBaseLink.position, targetPosition, Time.deltaTime * smoothSpeed);
            robotBaseLink.rotation = Quaternion.Lerp(robotBaseLink.rotation, targetRotation, Time.deltaTime * smoothSpeed);
        }

        // Update debug info
        if (robotBaseLink != null)
        {
            currentPosition = robotBaseLink.position;
            currentRotation = robotBaseLink.eulerAngles;
        }
    }

    // Helper methods for navigation scripts
    public Vector3 GetRobotPosition()
    {
        return robotBaseLink != null ? robotBaseLink.position : Vector3.zero;
    }

    public Vector3 GetRobotRotation()
    {
        return robotBaseLink != null ? robotBaseLink.eulerAngles : Vector3.zero;
    }

    public bool HasValidPosition()
    {
        return hasInitialPose;
    }
#endif
}
