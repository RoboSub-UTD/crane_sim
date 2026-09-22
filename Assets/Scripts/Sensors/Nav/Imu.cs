using UnityEngine;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Sim.Utils;
using Sim.Utils.ROS;

namespace Sim.Sensors.Nav {
    public class Imu : MonoBehaviour, IROSSensor<ImuMsg> {
        [SerializeField] private string topicName = "imu/raw";
        [SerializeField] private string frameId = "imu_link";
        [SerializeField] private float Hz = 50.0f;
        public ROSPublisher publisher { get; set; }

        public IPhysicsBody body;

        private Vector3 specificForce; // sensor frame, Unity axes
        private Vector3 prevVelocity;
        private bool velocityInitialised;

        private void OnValidate() {
            if (GetComponent<Rigidbody>() == null && GetComponent<ArticulationBody>() == null)
                Debug.LogWarning($"{name} should have either a Rigidbody or an ArticulationBody attached.");
        }

        private void Awake() {
            var rb = GetComponent<Rigidbody>();
            var ab = GetComponent<ArticulationBody>();

            if (rb != null) body = new RigidbodyAdapter(rb);
            else if (ab != null) body = new ArticulationBodyAdapter(ab);
            else throw new MissingComponentException($"{name} requires a Rigidbody or ArticulationBody!");

            publisher = gameObject.AddComponent<ROSPublisher>();
        }

        private void FixedUpdate() {
            Vector3 v = body.linearVelocity;
            Vector3 accelWorld = velocityInitialised ? (v - prevVelocity) / Time.fixedDeltaTime : Vector3.zero;
            prevVelocity = v;
            velocityInitialised = true;
            // What an accelerometer reads (a - g); at rest this is +g along the sensor's up axis (REP-145).
            specificForce = Quaternion.Inverse(body.transform.rotation) * (accelWorld - Physics.gravity);
        }

        public ImuMsg CreateMessage() {
            // sensor_msgs/Imu rates and accelerations are expressed in the sensor frame.
            Vector3 angVelLocal = Quaternion.Inverse(body.transform.rotation) * body.angularVelocity;
            Vector3 angVel = FLU.ConvertAngularVelocityFromRUF(angVelLocal);
            Vector3 accel = FLU.ConvertFromRUF(specificForce);
            return new ImuMsg {
                orientation = body.transform.rotation.To<FLU>(),
                angular_velocity = new Vector3Msg(angVel.x, angVel.y, angVel.z),
                linear_acceleration = new Vector3Msg(accel.x, accel.y, accel.z),
                header = publisher.CreateHeader()
            };
        }

        private void Start() {
            publisher.Initialize(topicName, frameId, CreateMessage, Hz);
        }
    }
}
