using RosMessageTypes.Vision;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Sim.Utils.ROS;
using UnityEngine;
using System.Collections.Generic;

namespace Sim.Sensors.Vision {
    [System.Serializable]
    public class ObjectEntry {
        public GameObject obj;
        public string id;
    }

    /// <summary>
    /// Publishes ground-truth 3D boxes of the listed objects visible from <see cref="sensorCamera"/>.
    /// Poses are in <see cref="frameId"/>, which must be the camera's own frame (REP-103: x out of
    /// the lens, y left, z up); TransformTreePublisher publishes it from the same camera transform.
    /// Box sizes are metric, along the box's own x/y/z in the same convention.
    /// </summary>
    public class BoundingBox3D : MonoBehaviour {
        [SerializeField] private string topicName = "/detections";
        [SerializeField] private string frameId = "front_camera_link";
        [SerializeField] private Camera sensorCamera;
        [SerializeField] private float Hz = 10f;

        [SerializeField] private float minDist = 1f;
        [SerializeField] private float maxDist = 20f;

        [SerializeField] private bool drawGizmos = true;
        [SerializeField] private float gizmoScale = 0.05f;

        private ROSPublisher publisher;
        private float timeSincePublish;

        [SerializeField] private List<ObjectEntry> objects = new();
        private Dictionary<GameObject, string> objectDict = new();

        private void Awake() {
            foreach (var entry in objects) {
                if (entry.obj != null && !string.IsNullOrEmpty(entry.id))
                    objectDict[entry.obj] = entry.id;
            }

            if (sensorCamera == null) {
                Debug.LogError("Missing camera reference.");
                enabled = false;
                return;
            }

            publisher = gameObject.AddComponent<ROSPublisher>();
        }

        private void Start() {
            publisher.Initialize(topicName, frameId, CreateMessage, Hz, true);
        }

        private void FixedUpdate() {
            timeSincePublish += Time.fixedDeltaTime;
            if (timeSincePublish >= 1f / Hz) {
                publisher.Publish();
                timeSincePublish = 0f;
            }
        }

        private void OnDrawGizmos() {
            if (!drawGizmos) return;

            foreach (var entry in objects) {
                if (entry.obj == null) continue;

                ComputeBounds(entry.obj, out Vector3 center, out Vector3 size);

                Transform t = entry.obj.transform;
                DrawBoundingBox(t.position + t.rotation * center, t.rotation, size);
            }
        }

        private void DrawBoundingBox(Vector3 center, Quaternion rotation, Vector3 size) {
            Vector3 half = size * 0.5f;

            Vector3[] localOffsets = new Vector3[]
            {
                new(-half.x, -half.y, -half.z),
                new( half.x, -half.y, -half.z),
                new( half.x, -half.y,  half.z),
                new(-half.x, -half.y,  half.z),

                new(-half.x,  half.y, -half.z),
                new( half.x,  half.y, -half.z),
                new( half.x,  half.y,  half.z),
                new(-half.x,  half.y,  half.z),
            };

            Vector3[] corners = new Vector3[8];

            for (int i = 0; i < 8; i++)
                corners[i] = center + rotation * localOffsets[i];

            Gizmos.color = Color.red;

            foreach (var c in corners)
                Gizmos.DrawSphere(c, gizmoScale);

            int[,] edges = {
                {0,1},{1,2},{2,3},{3,0},
                {4,5},{5,6},{6,7},{7,4},
                {0,4},{1,5},{2,6},{3,7}
            };

            for (int i = 0; i < 12; i++)
                Gizmos.DrawLine(corners[edges[i, 0]], corners[edges[i, 1]]);
        }

        private Detection3DArrayMsg CreateMessage() {
            List<Detection3DMsg> detections = new();

            foreach (var kvp in objectDict) {
                GameObject obj = kvp.Key;
                string id = kvp.Value;

                ComputeBounds(obj, out Vector3 center, out Vector3 size);

                Vector3 worldCenter = obj.transform.position + obj.transform.rotation * center;

                Vector3 screenPoint = sensorCamera.WorldToViewportPoint(worldCenter);
                bool visible =
                    screenPoint.z > 0 &&
                    screenPoint.x > 0 && screenPoint.x < 1 &&
                    screenPoint.y > 0 && screenPoint.y < 1;

                float dist = Vector3.Magnitude(worldCenter - sensorCamera.transform.position);
                bool inRange = minDist <= dist && dist <= maxDist;

                if (!visible || !inRange)
                    continue;

                // Box pose in the camera's axes (metric: no camera scale), then RUF -> FLU, the
                // same conversion the TF tree uses for the camera frame itself.
                Quaternion toCamera = Quaternion.Inverse(sensorCamera.transform.rotation);
                Vector3 cameraSpaceCenter = toCamera * (worldCenter - sensorCamera.transform.position);
                Quaternion cameraSpaceRotation = toCamera * obj.transform.rotation;

                PoseMsg pose = new(cameraSpaceCenter.To<FLU>(), cameraSpaceRotation.To<FLU>());
                // Extents along the box's own axes: Unity (right, up, forward) -> ROS (x=forward, y=left, z=up).
                Vector3Msg rosSize = new(size.z, size.x, size.y);

                detections.Add(GenerateDetection(pose, rosSize, id));
            }

            return new Detection3DArrayMsg(
                publisher.CreateHeader(),
                detections.ToArray()
            );
        }

        private Detection3DMsg GenerateDetection(
            PoseMsg pose,
            Vector3Msg size,
            string id) {
            double[] covariance = new double[36];

            ObjectHypothesisWithPoseMsg hypothesis =
                new(
                    new ObjectHypothesisMsg(id, 1.0f),
                    new PoseWithCovarianceMsg(pose, covariance)
                );

            BoundingBox3DMsg bbox = new(pose, size);

            return new Detection3DMsg(
                publisher.CreateHeader(),
                new[] { hypothesis },
                bbox,
                id
            );
        }

        /// <summary>
        /// Box around all of <paramref name="obj"/>'s renderers, aligned with the object's axes and in
        /// metres (every renderer's corners go through its own full transform, so scale and rotation
        /// anywhere in the hierarchy are accounted for). <paramref name="center"/> is the offset from
        /// the object's pivot in its axes; world centre = position + rotation * center.
        /// </summary>
        private static void ComputeBounds(
            GameObject obj,
            out Vector3 center,
            out Vector3 size) {
            Renderer[] renderers = obj.GetComponentsInChildren<Renderer>();

            if (renderers.Length == 0) {
                center = Vector3.zero;
                size = Vector3.zero;
                return;
            }

            Quaternion toObject = Quaternion.Inverse(obj.transform.rotation);
            Vector3 origin = obj.transform.position;
            Vector3 min = Vector3.positiveInfinity;
            Vector3 max = Vector3.negativeInfinity;

            foreach (var r in renderers) {
                Bounds b = r.localBounds;
                for (int i = 0; i < 8; i++) {
                    Vector3 corner = b.center + Vector3.Scale(b.extents, new Vector3(
                        (i & 1) == 0 ? -1f : 1f, (i & 2) == 0 ? -1f : 1f, (i & 4) == 0 ? -1f : 1f));
                    Vector3 p = toObject * (r.transform.TransformPoint(corner) - origin);
                    min = Vector3.Min(min, p);
                    max = Vector3.Max(max, p);
                }
            }

            center = (min + max) * 0.5f;
            size = max - min;
        }
    }
}

