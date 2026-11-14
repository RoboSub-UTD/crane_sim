using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using Sim.Utils.ROS;

namespace Sim.Sensors.Vision {
    public class ROSDepthCameraAsync : MonoBehaviour, IROSSensor<ImageMsg> {
        private static byte[] s_ScratchSpace;

        [SerializeField] private RenderTexture depthRenderTexture;
        [SerializeField] private Camera sensorCamera;

        [SerializeField] private string topicName = "camera/depth/image_rect_raw";
        [SerializeField] private string frameId = "front_camera_link";
        [SerializeField] private float Hz = 15.0f;
        public ROSPublisher publisher { get; set; }

        private CustomPassVolume customPassVolume;
        private CameraDepthBake depthBakePass = new();
        private Texture2D depthTex2D;
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
            sensorCamera = GetComponent<Camera>();
            customPassVolume = gameObject.AddComponent<CustomPassVolume>();
            customPassVolume.injectionPoint = CustomPassInjectionPoint.AfterPostProcess;
            customPassVolume.targetCamera = sensorCamera;
            depthBakePass.bakingCamera = sensorCamera;
            depthBakePass.depthTexture = depthRenderTexture;
            customPassVolume.customPasses.Add(depthBakePass);

            publisher.Initialize(topicName, frameId, CreateMessage, Hz, true);
        }

        public ImageMsg CreateMessage() {
            return GetDepthImageMsg(depthTex2D, publisher.CreateHeader(readbackStamp));
        }

        private void FixedUpdate() {
            timeSincePublish += Time.fixedDeltaTime;
            if (timeSincePublish > 1.0f / Hz) {
                RequestReadback(depthRenderTexture);
                timeSincePublish = 0.0f;
            }
        }

        private void RequestReadback(RenderTexture targetTexture) {
            pendingStamps.Enqueue(Clock.time);
            AsyncGPUReadback.Request(targetTexture, 0, TextureFormat.RFloat, OnReadbackComplete);
        }

        private void OnReadbackComplete(AsyncGPUReadbackRequest request) {
            readbackStamp = pendingStamps.Count > 0 ? pendingStamps.Dequeue() : Clock.time;
            if (request.hasError) {
                Debug.LogError("Failed to read back texture once");
                return;
            }

            if (depthTex2D == null || depthTex2D.width != depthRenderTexture.width || depthTex2D.height != depthRenderTexture.height) {
                depthTex2D = new Texture2D(depthRenderTexture.width, depthRenderTexture.height, TextureFormat.RFloat, false);
            }

            depthTex2D.LoadRawTextureData(request.GetData<byte>());
            depthTex2D.Apply();

            // Publish via ROSPublisher
            if (publisher != null) publisher.Publish();
        }

        private ImageMsg GetDepthImageMsg(Texture2D tex, HeaderMsg header) {
            byte[] data = null;
            string encoding = "32FC1";
            int step = 4 * tex.width;

            var floatData = tex.GetPixelData<float>(0).ToArray();
            data = new byte[floatData.Length * 4];
            Buffer.BlockCopy(floatData, 0, data, 0, data.Length);
            ReverseInBlocks(data, tex.width * 4, tex.height);
            return new ImageMsg(header, (uint)tex.height, (uint)tex.width, encoding, 0, (uint)step, data);
        }

        private void ReverseInBlocks(byte[] array, int blockSize, int numBlocks) {
            if (blockSize * numBlocks > array.Length) {
                Debug.LogError($"Invalid ReverseInBlocks, array length is {array.Length}, should be at least {blockSize * numBlocks}");
                return;
            }

            if (s_ScratchSpace == null || s_ScratchSpace.Length < blockSize)
                s_ScratchSpace = new byte[blockSize];

            int startBlockIndex = 0;
            int endBlockIndex = ((int)numBlocks - 1) * blockSize;

            while (startBlockIndex < endBlockIndex) {
                Buffer.BlockCopy(array, startBlockIndex, s_ScratchSpace, 0, blockSize);
                Buffer.BlockCopy(array, endBlockIndex, array, startBlockIndex, blockSize);
                Buffer.BlockCopy(s_ScratchSpace, 0, array, endBlockIndex, blockSize);
                startBlockIndex += blockSize;
                endBlockIndex -= blockSize;
            }
        }
    }
}
