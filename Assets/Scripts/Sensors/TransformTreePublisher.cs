using System;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Sim.Sensors.Zed;
using Sim.Utils.ROS;

namespace Sim.Sensors {
    /// <summary>
    /// Publishes the robot's TF tree (REP-105) below base_link on /tf:
    ///   base_link -> sensor frames        one per <see cref="sensorFrames"/> entry (imu_link, gps_link, ...)
    ///   base_link -> zed_camera_link      root of the static tree zed-ros2-wrapper publishes itself
    /// odom -> base_link is deliberately not published here; the state estimator owns it.
    ///
    /// Every frame is read from the live Unity transforms with the same RUF -> FLU conversion the
    /// sensors apply to their data, so the tree always agrees with what the sensors report. Each
    /// sensor entry must therefore point at the transform the sensor's data is expressed in.
    ///
    /// The fixed mounts go on /tf too, not /tf_static: ROS-TCP-Endpoint creates every publisher
    /// volatile (it ignores latch) while tf2 listeners subscribe to /tf_static transient-local, so a
    /// /tf_static published through the bridge is QoS-incompatible and never received.
    /// </summary>
    public class TransformTreePublisher : MonoBehaviour, IROSSensor<TFMessageMsg> {
        [Serializable]
        public struct SensorFrame {
            public string frameId;
            [Tooltip("Transform the sensor's data is expressed in (Unity +Z -> ROS x, -X -> y, +Y -> z).")]
            public Transform source;
        }

        [SerializeField] private string topicName = "/tf";
        [SerializeField] private float Hz = 50.0f;

        [Header("Body frame")]
        [SerializeField] private string baseFrameId = "base_link";
        [SerializeField] private Transform baseLink;
        [Tooltip("Rotation from the base link transform to the REP-103 body frame (x forward, y left, z up), " +
                 "as Unity euler angles: the rotated +Z must point out of the bow. Blastoise's URDF import has " +
                 "the bow along its local -X, hence (0, -90, 0).")]
        [SerializeField] private Vector3 baseFrameRotation = new Vector3(0f, -90f, 0f);

        [Header("Sensors")]
        [SerializeField] private List<SensorFrame> sensorFrames = new();
        [Tooltip("Attaches the zed-ros2-wrapper tree: base_link -> <zedCameraName>_camera_link.")]
        [SerializeField] private ZedSimCamera zedCamera;
        [Tooltip("The wrapper's camera_name launch argument.")]
        [SerializeField] private string zedCameraName = "zed";

        public ROSPublisher publisher { get; set; }

        private void Awake() {
            if (baseLink == null) baseLink = transform;
            publisher = gameObject.AddComponent<ROSPublisher>();
        }

        private void Start() {
            publisher.Initialize(topicName, baseFrameId, CreateMessage, Hz);
        }

        public TFMessageMsg CreateMessage() {
            var transforms = new List<TransformStampedMsg>(sensorFrames.Count + 1);
            Quaternion baseRot = baseLink.rotation * Quaternion.Euler(baseFrameRotation);
            Vector3 basePos = baseLink.position;

            foreach (SensorFrame sensor in sensorFrames) {
                if (sensor.source == null || string.IsNullOrEmpty(sensor.frameId)) continue;
                transforms.Add(Stamped(baseFrameId, sensor.frameId,
                    Relative(basePos, baseRot, sensor.source.position, sensor.source.rotation)));
            }

            if (zedCamera != null) {
                Pose centre = zedCamera.BodyCentre;
                // zed_camera_link is the mounting screw, straight below the body centre.
                Vector3 mount = centre.position - centre.rotation * new Vector3(0f, ZedCameraSpecs.MountToCentreHeight, 0f);
                transforms.Add(Stamped(baseFrameId, $"{zedCameraName}_camera_link",
                    Relative(basePos, baseRot, mount, centre.rotation)));
            }

            return new TFMessageMsg(transforms.ToArray());
        }

        /// <summary>Child pose in the parent's axes, converted to ROS (both frames RUF -> FLU).</summary>
        private static TransformMsg Relative(Vector3 parentPos, Quaternion parentRot, Vector3 childPos, Quaternion childRot) {
            Quaternion toParent = Quaternion.Inverse(parentRot);
            Vector3 translation = toParent * (childPos - parentPos);
            Quaternion rotation = toParent * childRot;
            return new TransformMsg(translation.To<FLU>(), rotation.To<FLU>());
        }

        private TransformStampedMsg Stamped(string parent, string child, TransformMsg tf) {
            var header = publisher.CreateHeader();
            header.frame_id = parent;
            return new TransformStampedMsg(header, child, tf);
        }
    }
}
