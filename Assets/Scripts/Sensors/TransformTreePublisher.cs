using System;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Sim.Utils.ROS;

namespace Sim.Sensors {
    public class TransformTreePublisher : MonoBehaviour, IROSSensor<TFMessageMsg> {
        [Serializable]
        public struct SensorFrame {
            public string frameId;
            public Transform source;
        }

        [SerializeField] private string topicName = "/tf";
        [SerializeField] private float Hz = 50.0f;

        [SerializeField] private string baseFrameId = "base_link";
        [SerializeField] private Transform baseLink;
        [SerializeField] private Vector3 baseFrameRotation = new Vector3(0f, -90f, 0f);

        [SerializeField] private List<SensorFrame> sensorFrames = new();

        public ROSPublisher publisher { get; set; }

        private void Awake() {
            if (baseLink == null) baseLink = transform;
            publisher = gameObject.AddComponent<ROSPublisher>();
        }

        private void Start() {
            publisher.Initialize(topicName, baseFrameId, CreateMessage, Hz);
        }

        public TFMessageMsg CreateMessage() {
            var transforms = new List<TransformStampedMsg>(sensorFrames.Count);
            Quaternion baseRot = baseLink.rotation * Quaternion.Euler(baseFrameRotation);
            Vector3 basePos = baseLink.position;

            foreach (SensorFrame sensor in sensorFrames) {
                if (sensor.source == null || string.IsNullOrEmpty(sensor.frameId)) continue;

                transforms.Add(Stamped(
                    baseFrameId,
                    sensor.frameId,
                    Relative(
                        basePos,
                        baseRot,
                        sensor.source.position,
                        sensor.source.rotation
                    )
                ));
            }

            return new TFMessageMsg(transforms.ToArray());
        }

        private static TransformMsg Relative(
            Vector3 parentPos,
            Quaternion parentRot,
            Vector3 childPos,
            Quaternion childRot
        ) {
            Quaternion toParent = Quaternion.Inverse(parentRot);
            Vector3 translation = toParent * (childPos - parentPos);
            Quaternion rotation = toParent * childRot;

            return new TransformMsg(
                translation.To<FLU>(),
                rotation.To<FLU>()
            );
        }

        private TransformStampedMsg Stamped(
            string parent,
            string child,
            TransformMsg tf
        ) {
            var header = publisher.CreateHeader();
            header.frame_id = parent;

            return new TransformStampedMsg(
                header,
                child,
                tf
            );
        }
    }
}
