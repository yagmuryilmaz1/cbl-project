using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;
using Codice.Client.BaseCommands.WkStatus.Printers;

namespace RosSharp.Control
{
    public enum ControlMode { Keyboard, ROS, Navigation };

    public class AGVController : MonoBehaviour
    {
        [Header("Robot Configuration")]
        public GameObject wheel1;
        public GameObject wheel2;
        public ControlMode mode = ControlMode.ROS;

        [Header("Movement Control")]
        public bool usePhysicsMovement = false; // SET THIS TO FALSE for position tracking

        [Header("Physics Parameters")]
        public float maxLinearSpeed = 2; //  m/s
        public float maxRotationalSpeed = 1;//
        public float wheelRadius = 0.033f; //meters
        public float trackWidth = 0.288f; // meters Distance between tyres
        public float forceLimit = 10;
        public float damping = 10;

        [Header("ROS Settings")]
        public float ROSTimeout = 0.5f;

        [Header("Navigation Control")]
        public bool enableAutonomousControl = false;

        private ArticulationBody wA1;
        private ArticulationBody wA2;
        private float lastCmdReceived = 0f;

        ROSConnection ros;
        private RotationDirection direction;
        private float rosLinear = 0f;
        private float rosAngular = 0f;

        // Navigation control variables
        private float autonomousLinear = 0f;
        private float autonomousAngular = 0f;

        void Start()
        {
            // Initialize wheel physics only if using physics movement
            if (usePhysicsMovement)
            {
                wA1 = wheel1.GetComponent<ArticulationBody>();
                wA2 = wheel2.GetComponent<ArticulationBody>();
                SetParameters(wA1);
                SetParameters(wA2);
            }

            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSCmd);

            Debug.Log($"[AGVController] Started - Physics Movement: {usePhysicsMovement}");
        }

        void ReceiveROSCmd(TwistMsg cmdVel)
        {
            if (cmdVel.linear.x == 0 && cmdVel.angular.z == 0)
            {
                rosLinear = 0;
                rosAngular = 0;
            }
            else if (cmdVel.linear.x < 0)
            {
                rosLinear = (float)((float)cmdVel.linear.x - 0.02);
            }
            else if (cmdVel.angular.z < 0)
            {
                rosAngular = (float)((float)cmdVel.angular.z - 0.18);
            }
            else if (cmdVel.linear.x > 0)
            {
                rosLinear = (float)((float)cmdVel.linear.x + 0.02); //(float)cmdVel.linear.x ;//added 0.05 to account for friction 
            }
            else if (cmdVel.angular.z > 0)
            {
                rosAngular = (float)((float)cmdVel.angular.z + 0.18); //(float)cmdVel.angular.z; //added 0.03 to acct for friction
            }
            lastCmdReceived = Time.time;
        }

        void FixedUpdate()
        {
            if (mode == ControlMode.Keyboard)
            {
                KeyBoardUpdate();
            }
            else if (mode == ControlMode.ROS)
            {
                ROSUpdate();
            }
            else if (mode == ControlMode.Navigation)
            {
                NavigationUpdate();
            }
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }
            joint.xDrive = drive;
        }

        private void KeyBoardUpdate()
        {
            float moveDirection = Input.GetAxis("Vertical");
            float inputSpeed;
            float inputRotationSpeed;

            if (moveDirection > 0)
            {
                inputSpeed = maxLinearSpeed;
            }
            else if (moveDirection < 0)
            {
                inputSpeed = maxLinearSpeed * -1;
            }
            else
            {
                inputSpeed = 0;
            }

            float turnDirection = Input.GetAxis("Horizontal");
            if (turnDirection > 0)
            {
                inputRotationSpeed = maxRotationalSpeed;
            }
            else if (turnDirection < 0)
            {
                inputRotationSpeed = maxRotationalSpeed * -1;
            }
            else
            {
                inputRotationSpeed = 0;
            }

            RobotInput(inputSpeed, inputRotationSpeed);
        }

        private void ROSUpdate()
        {
            if (!usePhysicsMovement)
            {
                // Don't move Unity robot - RobotPositionTracker handles position
                // Just forward any autonomous commands to real robot
                if (enableAutonomousControl)
                {
                    SendCommandToRealRobot(autonomousLinear, autonomousAngular);
                }
                return;
            }

            // Original physics-based movement code
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear = 0f;
                rosAngular = 0f;
            }

            Debug.Log("linear: " + rosLinear);
            Debug.Log("angular: " + rosAngular);
            RobotInput(rosLinear, -rosAngular);
        }

        private void NavigationUpdate()
        {
            // For autonomous navigation mode
            if (enableAutonomousControl)
            {
                if (usePhysicsMovement)
                {
                    RobotInput(autonomousLinear, autonomousAngular);
                }
                else
                {
                    SendCommandToRealRobot(autonomousLinear, autonomousAngular);
                }
            }
        }

        private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
        {
            if (!usePhysicsMovement) return; // Skip if not using physics

            if (speed > maxLinearSpeed)
            {
                speed = maxLinearSpeed;
            }
            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }

            float wheel1Rotation = (speed / wheelRadius);
            float wheel2Rotation = wheel1Rotation;
            float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);

            if (rotSpeed != 0)
            {
                wheel1Rotation = (wheel1Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                wheel2Rotation = (wheel2Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
            }
            else
            {
                wheel1Rotation *= Mathf.Rad2Deg;
                wheel2Rotation *= Mathf.Rad2Deg;
            }

            SetSpeed(wA1, wheel1Rotation);
            SetSpeed(wA2, wheel2Rotation);
        }

        // PUBLIC METHODS FOR NAVIGATION SCRIPTS

        /// ‡‡<summary>_PLACEHOLDER‡‡
        /// Send movement command for autonomous navigation
        /// ‡‡</summary>_PLACEHOLDER‡‡
        public void SendNavigationCommand(float linear, float angular)
        {
            autonomousLinear = linear;
            autonomousAngular = angular;
            enableAutonomousControl = true;

            // Always send to real robot
            SendCommandToRealRobot(linear, angular);

            Debug.Log($"[AGVController] Navigation Command - Linear: {linear:F2}, Angular: {angular:F2}");
        }

        /// ‡‡<summary>_PLACEHOLDER‡‡
        /// Stop autonomous navigation
        /// ‡‡</summary>_PLACEHOLDER‡‡
        public void StopNavigation()
        {
            autonomousLinear = 0f;
            autonomousAngular = 0f;
            enableAutonomousControl = false;
            SendCommandToRealRobot(0f, 0f);
        }

        /// ‡‡<summary>_PLACEHOLDER‡‡
        /// Send command directly to real robot via ROS
        /// ‡‡</summary>_PLACEHOLDER‡‡
        private void SendCommandToRealRobot(float linear, float angular)
        {
            TwistMsg twist = new TwistMsg();
            twist.linear.x = linear;
            twist.angular.z = angular;
            ros.Publish("cmd_vel", twist);
        }

        /// ‡‡<summary>_PLACEHOLDER‡‡
        /// Check if robot is currently moving
        /// ‡‡</summary>_PLACEHOLDER‡‡
        public bool IsMoving()
        {
            return Mathf.Abs(autonomousLinear) > 0.01f || Mathf.Abs(autonomousAngular) > 0.01f;
        }

        /// ‡‡<summary>_PLACEHOLDER‡‡
        /// Get current command values
        /// ‡‡</summary>_PLACEHOLDER‡‡
        public Vector2 GetCurrentCommand()
        {
            return new Vector2(autonomousLinear, autonomousAngular);
        }
    }
}
