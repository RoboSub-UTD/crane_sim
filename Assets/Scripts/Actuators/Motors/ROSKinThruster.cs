using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.GalaxseaInterfaces;

namespace Sim.Actuators.Motors {
    public class ROSKinThruster : MonoBehaviour {
        public enum ThrusterPosition {
            FrontLeft,
            FrontRight,
            BackLeft,
            BackRight
        }

        public enum Axis {
            X,
            Y,
            Z
        }

        [Header("ROS Topic & Selection")]
        [SerializeField] private string topicName = "/thruster_commands";
        [SerializeField] private ThrusterPosition position;

        [Header("Direction Calibration")]
        [Tooltip("Check this if positive commands move the vessel backward instead of forward.")]
        [SerializeField] private bool invertThrustDirection = true;

        [Header("Physical Thrust Force")]
        [SerializeField] private float maxThrustForceN = 100f;
        [SerializeField] private float backK = 0.8f;
        [SerializeField] private Transform forcePoint;

        [Header("Visual Propeller Settings")]
        [SerializeField] private Transform propellerMesh;
        [SerializeField] private float maxVisualRPM = 1200f;
        [SerializeField] private Axis rotationAxis = Axis.Z;

        [Header("Debug")]
        [SerializeField] private bool showDebugRay = true;

        private ROSConnection ros;
        private Rigidbody targetRigidbody;
        private ArticulationBody targetArticulationBody;

        private float currentCommand = 0f;
        private Vector3 localPropellerAxis;

        private void Awake() {
            Transform rootTransform = transform.root;

            targetArticulationBody =
                rootTransform.GetComponentInChildren<ArticulationBody>();

            if (targetArticulationBody == null) {
                targetRigidbody =
                    rootTransform.GetComponentInChildren<Rigidbody>();
            }

            if (targetRigidbody == null && targetArticulationBody == null) {
                Debug.LogError(
                    $"{name}: Could not find an ArticulationBody or Rigidbody on root structure!"
                );
            }

            if (forcePoint == null) {
                forcePoint = transform;
            }

            localPropellerAxis = GetAxisVector(rotationAxis);
        }

        private void Start() {
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<ThrusterCommandsMsg>(topicName, CommandCallback);
        }

        private Vector3 GetAxisVector(Axis axis) {
            return axis switch {
                Axis.X => Vector3.right,
                Axis.Y => Vector3.up,
                Axis.Z => Vector3.forward,
                _ => Vector3.forward
            };
        }

        private void CommandCallback(ThrusterCommandsMsg msg) {
            currentCommand = position switch {
                ThrusterPosition.FrontLeft =>
                    (float)msg.front_left,

                ThrusterPosition.FrontRight =>
                    (float)msg.front_right,

                ThrusterPosition.BackLeft =>
                    (float)msg.back_left,

                ThrusterPosition.BackRight =>
                    (float)msg.back_right,

                _ => 0f
            };
        }

        private void Update() {
            if (propellerMesh != null &&
                !Mathf.Approximately(currentCommand, 0f)) {

                float visualCommand =
                    Mathf.Clamp(currentCommand, -1f, 1f);

                float degPerSec =
                    visualCommand * (maxVisualRPM * 6f);

                propellerMesh.Rotate(
                    localPropellerAxis,
                    degPerSec * Time.deltaTime,
                    Space.Self
                );
            }
        }

        private void FixedUpdate() {
            if (Mathf.Abs(currentCommand) < 0.001f) {
                return;
            }

            Vector3 forceDirection = transform.up;

            if (invertThrustDirection) {
                forceDirection = -forceDirection;
            }

            float signedThrustForce = currentCommand * maxThrustForceN;

            if (currentCommand < 0f) {
                signedThrustForce *= backK;
            }

            Vector3 appliedForce =
                forceDirection * signedThrustForce;

            if (targetArticulationBody != null) {
                targetArticulationBody.WakeUp();

                targetArticulationBody.AddForceAtPosition(
                    appliedForce,
                    forcePoint.position,
                    ForceMode.Force
                );
            }
            else if (targetRigidbody != null) {
                targetRigidbody.WakeUp();

                targetRigidbody.AddForceAtPosition(
                    appliedForce,
                    forcePoint.position,
                    ForceMode.Force
                );
            }

            if (showDebugRay) {
                Debug.DrawRay(
                    forcePoint.position,
                    appliedForce * 0.05f,
                    Color.cyan,
                    Time.fixedDeltaTime
                );
            }
            
            
        }
    }
}


