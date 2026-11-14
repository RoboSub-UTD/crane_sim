using RosMessageTypes.Sensor;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Sim.Utils.ROS;

namespace Sim.Sensors.Vision {
    public class ROSCameraAsync : MonoBehaviour, IROSSensor<ImageMsg> {
        [SerializeField] private RenderTexture rgbRenderTexture;
        [SerializeField] private Camera sensorCamera;

        [SerializeField] private string topicName = "camera/image_raw";
        [SerializeField] private string frameId = "front_camera_link";
        [SerializeField] private float Hz = 15.0f;
        public ROSPublisher publisher { get; set; }

        private Texture2D rgbTexture2D;
        private float timeSincePublish = 0.0f;
        // Sim time at which each pending readback was requested (readbacks complete in request order)
        private readonly Queue<double> pendingStamps = new();
        private double readbackStamp;

        private void Awake() {
            if (sensorCamera == null) {
                Debug.LogError("Missing a camera reference.");
                enabled = false;
                return;
            }

            publisher = gameObject.AddComponent<ROSPublisher>();
        }

        private void Start() {
            sensorCamera.targetTexture = rgbRenderTexture;
            publisher.Initialize(topicName, frameId, CreateMessage, Hz, true);
        }

        public ImageMsg CreateMessage() {
            return rgbTexture2D.ToImageMsg(publisher.CreateHeader(readbackStamp));
        }

        private void FixedUpdate() {
            timeSincePublish += Time.fixedDeltaTime;
            if (timeSincePublish > 1.0f / Hz) {
                RequestReadback(rgbRenderTexture);
                timeSincePublish = 0.0f;
            }
        }

        private void RequestReadback(RenderTexture targetTexture) {
            pendingStamps.Enqueue(Clock.time);
            AsyncGPUReadback.Request(targetTexture, 0, TextureFormat.RGB24, OnReadbackComplete);
        }

        private void OnReadbackComplete(AsyncGPUReadbackRequest request) {
            readbackStamp = pendingStamps.Count > 0 ? pendingStamps.Dequeue() : Clock.time;
            if (request.hasError) {
                Debug.LogWarning("Failed to read back texture once");
                return;
            }

            if (rgbTexture2D == null || rgbTexture2D.width != sensorCamera.targetTexture.width || rgbTexture2D.height != sensorCamera.targetTexture.height) {
                rgbTexture2D = new Texture2D(sensorCamera.targetTexture.width, sensorCamera.targetTexture.height, TextureFormat.RGB24, false);
            }

            rgbTexture2D.LoadRawTextureData(request.GetData<byte>());
            rgbTexture2D.Apply();

            if (publisher != null) publisher.Publish();
        }
    }
}
