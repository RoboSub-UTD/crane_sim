using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using Sim.Utils;

namespace Sim.Sensors.Zed {
    /// <summary>
    /// Simulated ZED 2i that streams a rectified stereo pair plus IMU into the real ZED SDK through
    /// Stereolabs' simulation streamer. Nothing is published to ROS from here:
    /// run zed-ros2-wrapper with <c>sim_mode:=true</c> against this machine and it produces the
    /// usual /zed/zed_node/... topics (images, SDK-computed depth, IMU).
    ///
    /// Attach to a link on the robot. Either give it an existing <see cref="leftCamera"/> or leave it
    /// empty and set <see cref="rigOrigin"/>; either way that transform marks the centre of the camera
    /// body (where the ZED2i mesh sits), and the two eyes are placed on the real camera's optical
    /// centres around it: ±60 mm along its X and 10 mm behind it. Unity +Z of the eye transforms looks
    /// out of the lenses, +Y is up.
    /// </summary>
    public class ZedSimCamera : MonoBehaviour {
        [Header("Rig")]
        [Tooltip("Optional existing camera to reuse as the left eye (its FOV/clip planes/target are overridden). It is moved onto the left optical centre, so place it at the centre of the camera body.")]
        [SerializeField] private Camera leftCamera;
        [Tooltip("Used only when no left camera is given: centre of the camera body, eyes spawned either side. Defaults to this transform.")]
        [SerializeField] private Transform rigOrigin;
        [SerializeField] private ZedResolution resolution = ZedResolution.HD720;
        [SerializeField] private ZedLens lens = ZedLens.Wide;
        [SerializeField, Range(1, 60)] private int fps = 30;
        [SerializeField] private float nearClip = 0.3f;
        [SerializeField] private float farClip = 1000f;
        [Tooltip("GPU readbacks come out bottom-up on most platforms; the streamer wants top-down. Untick if the image is upside down in the wrapper.")]
        [SerializeField] private bool flipVertically = true;

        [Header("Streamer")]
        [SerializeField] private ushort port = 30000;
        [Tooltip("0 = pick automatically from the SDK's virtual ZED 2i serial pool.")]
        [SerializeField] private int serialNumber = 0;
        [Tooltip("0 = pick a default for the resolution.")]
        [SerializeField] private int bitrateKbps = 0;
        [SerializeField] private bool verboseStreamer = false;

        [Header("Depth check")]
        [Tooltip("Periodically log the true distance to the scene at a few pixels. Compare with the wrapper's depth at the same pixels (Docker/zed/depth_probe.py in roboboat-docker) to confirm the cloud is metric.")]
        [SerializeField] private bool logGroundTruthDepth = false;
        [SerializeField, Min(0.1f)] private float groundTruthInterval = 2f;

        [Header("IMU")]
        [Tooltip("Also push IMU samples every physics step through ingest_imu (angular velocity included). Experimental; the per-frame IMU is always sent.")]
        [SerializeField] private bool ingestHighRateImu = false;

        // One id per rig instance; the native registry lives for the whole editor process.
        private static int nextStreamerId;
        private int StreamerId;
        private const int SlotCount = 3;

        private ZedCameraSpecs.Spec spec;
        private Camera leftEye, rightEye;
        private RenderTexture leftRT, rightRT, leftFlipRT, rightFlipRT;

        private IPhysicsBody body;
        private Vector3 prevVelocity;
        private bool velocityInitialised;
        // Latest IMU sample, already in the streamer's frame (X left, Y up, Z forward, right-handed).
        private Quaternion imuOrientation = Quaternion.identity;
        private Vector3 imuSpecificForce;
        private Vector3 imuAngularVelocity;

        private bool streamerReady;
        private double lastCaptureTime = double.NegativeInfinity;
        private double lastGroundTruthTime = double.NegativeInfinity;
        private long epochBaseNs;
        private double simTimeBase;
        private bool timeBaseSet;
        private bool startedOnce;

        // Frame hand-off: readback callbacks (main thread) fill a slot, the worker encodes+sends it.
        private class FrameSlot {
            public byte[] left, right;
            public GCHandle leftHandle, rightHandle;
            public long timestampNs;
            public Quaternion orientation;
            public Vector3 specificForce;
            public int pendingReadbacks;
            public bool failed;
        }

        /// <summary>Where <see cref="LogGroundTruthDepth"/> samples, as fractions of the image.</summary>
        private static readonly Vector2[] ProbePixels = {
            new Vector2(0.5f, 0.5f), new Vector2(0.25f, 0.5f), new Vector2(0.75f, 0.5f),
            new Vector2(0.5f, 0.25f), new Vector2(0.5f, 0.75f),
        };

        private readonly FrameSlot[] slots = new FrameSlot[SlotCount];
        private readonly Stack<FrameSlot> freeSlots = new();
        private FrameSlot readySlot;
        private readonly object sync = new();
        private Thread worker;
        private bool stopping;

        /// <summary>
        /// World pose of the centre of the camera body, i.e. the wrapper's zed_camera_center
        /// (Unity +Z out of the lenses, +Y up). Valid both before and after Awake has slid a reused
        /// left camera onto the left optical centre.
        /// </summary>
        public Pose BodyCentre {
            get {
                if (leftCamera != null) {
                    Transform eye = leftCamera.transform;
                    Vector3 centre = leftEye != null ? eye.position - eye.rotation * ZedCameraSpecs.LeftEyeOffset : eye.position;
                    return new Pose(centre, eye.rotation);
                }
                Transform origin = rigOrigin != null ? rigOrigin : transform;
                return new Pose(origin.position, origin.rotation);
            }
        }

        private void Awake() {
            StreamerId = nextStreamerId++;
            spec = ZedCameraSpecs.Get(resolution, lens);

            var ab = GetComponentInParent<ArticulationBody>();
            var rb = GetComponentInParent<Rigidbody>();
            if (ab != null) body = new ArticulationBodyAdapter(ab);
            else if (rb != null) body = new RigidbodyAdapter(rb);
            else Debug.LogWarning($"{name}: no ArticulationBody/Rigidbody in parents, IMU will report a static camera.");

            leftRT = CreateEyeTexture("zed_left_rt");
            rightRT = CreateEyeTexture("zed_right_rt");
            leftFlipRT = CreateEyeTexture("zed_left_flip_rt");
            rightFlipRT = CreateEyeTexture("zed_right_flip_rt");

            if (leftCamera != null) {
                // The given camera marks the body centre, not the left lens: slide it onto the left
                // optical centre (world space, so no parent scale can shrink the offset) and hang the
                // right eye off it. The left eye stays the SDK's reference frame.
                Transform eye = leftCamera.transform;
                eye.position += eye.rotation * ZedCameraSpecs.LeftEyeOffset;
                rigOrigin = eye;
                leftEye = ConfigureEye(leftCamera, leftRT);
                rightEye = SpawnEye("zed_right_camera", new Vector3(ZedCameraSpecs.Baseline, 0f, 0f), rightRT);
            } else {
                if (rigOrigin == null) rigOrigin = transform;
                leftEye = SpawnEye("zed_left_camera", ZedCameraSpecs.LeftEyeOffset, leftRT);
                rightEye = SpawnEye("zed_right_camera", ZedCameraSpecs.RightEyeOffset, rightRT);
            }

            int bytes = spec.width * spec.height * 4;
            for (int i = 0; i < SlotCount; i++) {
                var slot = new FrameSlot { left = new byte[bytes], right = new byte[bytes] };
                slot.leftHandle = GCHandle.Alloc(slot.left, GCHandleType.Pinned);
                slot.rightHandle = GCHandle.Alloc(slot.right, GCHandleType.Pinned);
                slots[i] = slot;
                freeSlots.Push(slot);
            }
        }

        private RenderTexture CreateEyeTexture(string rtName) {
            var rt = new RenderTexture(spec.width, spec.height, 24, RenderTextureFormat.ARGB32) { name = rtName };
            rt.Create();
            return rt;
        }

        /// <summary>Spawns an eye at <paramref name="offset"/> metres from <see cref="rigOrigin"/>, in its axes.</summary>
        private Camera SpawnEye(string eyeName, Vector3 offset, RenderTexture target) {
            var go = new GameObject(eyeName);
            var cam = go.AddComponent<Camera>();
            var hdData = go.AddComponent<HDAdditionalCameraData>();
            if (leftCamera != null) {
                // Both eyes must render identically (exposure, AA, culling...) or stereo matching suffers.
                // Camera.CopyFrom also moves this transform onto the left camera, so the eye is
                // placed afterwards; doing it before collapses the baseline to zero and the SDK
                // then stereo-matches the left image against itself.
                cam.CopyFrom(leftCamera);
                var leftHd = leftCamera.GetComponent<HDAdditionalCameraData>();
                if (leftHd != null) leftHd.CopyTo(hdData);
            }

            go.transform.SetParent(rigOrigin, false);
            // localPosition is in the parent's units: divide out its scale so the baseline stays
            // metric on a scaled robot. Depth comes straight off the baseline, so a factor here is a
            // factor on every point in the cloud (the start-up check below catches what slips past).
            Vector3 scale = rigOrigin.lossyScale;
            go.transform.localPosition = new Vector3(
                offset.x / NonZero(scale.x), offset.y / NonZero(scale.y), offset.z / NonZero(scale.z));
            go.transform.localRotation = Quaternion.identity;
            return ConfigureEye(cam, target);
        }

        private static float NonZero(float v) => Mathf.Abs(v) < 1e-6f ? 1e-6f : v;

        private Camera ConfigureEye(Camera cam, RenderTexture target) {
            cam.usePhysicalProperties = false;
            cam.fieldOfView = spec.VerticalFov;
            cam.nearClipPlane = nearClip;
            cam.farClipPlane = farClip;
            cam.targetTexture = target;
            // After the target, which would otherwise recompute it: pin the aspect to the sensor's own
            // so nothing (game view, dynamic resolution) can bend fx away from fy, and with it the
            // disparity the SDK turns into depth.
            cam.aspect = (float)spec.width / spec.height;
            cam.allowHDR = true;
            cam.enabled = true;
            return cam;
        }

        private void Start() {
            startedOnce = true;
            StartStreaming();
        }

        private void OnEnable() {
            // Start() handles the first activation; this covers re-enabling in the inspector.
            if (startedOnce && !streamerReady) StartStreaming();
        }

        private void StartStreaming() {
            int structSize = Marshal.SizeOf<ZedSimNative.StreamingParameters>();
            if (structSize != 100) {
                Debug.LogError($"ZedSimCamera: StreamingParameters marshals to {structSize} bytes, expected 100. Binding out of sync with libsl_zed.");
                enabled = false;
                return;
            }

            Version sdk = ZedSimNative.TryGetRuntimeVersion();
            if (sdk == null) {
                Debug.LogError("ZedSimCamera: libsl_zed not found. Run Tools ▸ ZED Sim ▸ Install streamer library (and `sudo apt install libturbojpeg` on Linux).");
                enabled = false;
                return;
            }

            if (!VerifyRigMatchesCalibration()) {
                enabled = false;
                return;
            }

            int serial = serialNumber;
            if (serial == 0) serial = ZedSimNative.PickVirtualSerial(ZedCameraSpecs.Zed2iModelId, ZedCameraSpecs.ToSimLens(lens));
            if (serial == 0 || !ZedSimNative.IsSerialValid(serial)) {
                Debug.LogError($"ZedSimCamera: no valid virtual ZED 2i serial (requested {serialNumber}).");
                enabled = false;
                return;
            }

            var p = ZedSimNative.StreamingParameters.Default();
            p.imageWidth = spec.width;
            p.imageHeight = spec.height;
            p.fps = fps;
            p.port = port;
            p.serialNumber = serial;
            // Buffer bytes are genuinely R,G,B,A (AsyncGPUReadback converts to RGBA32 explicitly),
            // but libsl_zed's INPUT_FORMAT enum isn't vendored here to check our RGB/BGR mapping
            // against, so this value is unverified against the real header.
            p.inputFormat = ZedSimNative.InputFormat.BGR;
            p.bitrateKbps = bitrateKbps > 0 ? bitrateKbps : ZedCameraSpecs.DefaultBitrateKbps(resolution);
            p.verbose = (byte)(verboseStreamer ? 1 : 0);

            int result = ZedSimNative.InitStreamer(StreamerId, ref p);
            if (result == -1) {
                // The id is still registered from an earlier session in this editor process
                // (e.g. a crash, or a failed init): dispose it and retry once.
                ZedSimNative.CloseStreamer(StreamerId);
                result = ZedSimNative.InitStreamer(StreamerId, ref p);
            }
            if (result != 1) {
                // libsl_zed keeps the registration even when the RTP session fails (zed-isaac-sim #57);
                // release it or every later init in this process reports the id as in use.
                ZedSimNative.CloseStreamer(StreamerId);
                Debug.LogError($"ZedSimCamera: init_streamer failed: {ZedSimNative.DescribeInitResult(result)}");
                enabled = false;
                return;
            }

            streamerReady = true;
            stopping = false;
            Debug.Log($"ZED streamer ready: SDK {sdk}, ZED2i serial {serial}, {spec.width}x{spec.height}@{fps} " +
                $"(fx {spec.fx:F1}, vFOV {spec.VerticalFov:F1}°, baseline {MeasuredBaseline * 1000f:F2} mm), port {port}");

            worker = new Thread(WorkerLoop) { Name = "ZedSimStreamer", IsBackground = true };
            worker.Start();
        }

        private float MeasuredBaseline => Vector3.Distance(leftEye.transform.position, rightEye.transform.position);

        /// <summary>
        /// Checks the rig against the calibration the SDK has already assigned to this virtual serial.
        /// Depth is computed from disparity with <i>that</i> calibration, not with anything the
        /// streamer sends, so a rig that disagrees produces a cloud whose every point is wrong by the
        /// same factor — silently, and only in the metres, not in the picture. Refuse to stream
        /// instead of publishing a plausible-looking cloud at the wrong scale.
        /// </summary>
        private bool VerifyRigMatchesCalibration() {
            float baseline = MeasuredBaseline;
            if (Mathf.Abs(baseline - ZedCameraSpecs.Baseline) > 5e-4f) {
                Debug.LogError($"ZedSimCamera: the eyes are {baseline * 1000f:F2} mm apart but the SDK calibrates " +
                    $"this camera at {ZedCameraSpecs.Baseline * 1000f:F2} mm, so every depth would come out scaled " +
                    $"by {ZedCameraSpecs.Baseline / Mathf.Max(baseline, 1e-6f):F3}. Check the scale on the camera's parents.");
                return false;
            }

            // Rectified stereo: the eyes must be parallel and separated along the left eye's +X, or
            // the SDK matches along the wrong epipolar line and depth degrades or disappears.
            float twist = Quaternion.Angle(leftEye.transform.rotation, rightEye.transform.rotation);
            if (twist > 0.01f) {
                Debug.LogError($"ZedSimCamera: the eyes are toed in by {twist:F3}°; the SDK assumes a rectified pair.");
                return false;
            }
            Vector3 leftToRight = (rightEye.transform.position - leftEye.transform.position).normalized;
            if (Vector3.Dot(leftToRight, leftEye.transform.right) < 0.9999f) {
                Debug.LogError("ZedSimCamera: the right eye is not on the left eye's +X axis (rigOrigin rotated?).");
                return false;
            }

            return VerifyEyeIntrinsics("left", leftEye) && VerifyEyeIntrinsics("right", rightEye);
        }

        /// <summary>Reads fx/fy/cx/cy back out of the eye's own projection matrix and compares with the spec.</summary>
        private bool VerifyEyeIntrinsics(string label, Camera eye) {
            Matrix4x4 m = eye.projectionMatrix;
            float fx = 0.5f * spec.width * m.m00;
            float fy = 0.5f * spec.height * m.m11;
            float cx = 0.5f * spec.width * (1f - m.m02);
            float cy = 0.5f * spec.height * (1f - m.m12);

            if (Mathf.Abs(fx - spec.fx) > 0.001f * spec.fx || Mathf.Abs(fy - spec.fx) > 0.001f * spec.fx) {
                Debug.LogError($"ZedSimCamera: {label} eye renders at fx {fx:F2}, fy {fy:F2}, but the SDK rectifies " +
                    $"it as fx = fy = {spec.fx:F2}. Depth would scale with the ratio.");
                return false;
            }
            if (Mathf.Abs(cx - spec.width / 2f) > 0.5f || Mathf.Abs(cy - spec.height / 2f) > 0.5f) {
                Debug.LogError($"ZedSimCamera: {label} eye's principal point is ({cx:F1}, {cy:F1}), not the " +
                    $"({spec.width / 2f:F1}, {spec.height / 2f:F1}) the SDK assumes (oblique or physical camera?).");
                return false;
            }
            return true;
        }

        /// <summary>
        /// Logs the true distance to the scene at a few pixels, as the SDK measures it: Z along the
        /// left eye's optical axis, not range. Compare with depth_registered at the same pixels.
        /// </summary>
        private void LogGroundTruthDepth() {
            var line = new System.Text.StringBuilder("ZED ground-truth Z (metres):");
            foreach (var uv in ProbePixels) {
                int u = Mathf.RoundToInt(uv.x * spec.width);
                int v = Mathf.RoundToInt(uv.y * spec.height);
                // Image rows run top-down; Unity's screen coordinates start at the bottom left.
                Ray ray = leftEye.ScreenPointToRay(new Vector3(u + 0.5f, spec.height - 0.5f - v, 0f));
                line.Append(Physics.Raycast(ray, out RaycastHit hit, farClip)
                    ? $"  ({u},{v})={Vector3.Dot(hit.point - leftEye.transform.position, leftEye.transform.forward):F3}"
                    : $"  ({u},{v})=miss");
            }
            Debug.Log(line.ToString());
        }

        private void FixedUpdate() {
            SampleImu();
            if (ingestHighRateImu && streamerReady) {
                ZedSimNative.IngestImu(StreamerId, CurrentTimestampNs(),
                    imuAngularVelocity.x, imuAngularVelocity.y, imuAngularVelocity.z,
                    imuSpecificForce.x, imuSpecificForce.y, imuSpecificForce.z,
                    imuOrientation.w, imuOrientation.x, imuOrientation.y, imuOrientation.z);
            }
        }

        /// <summary>
        /// Derives orientation, specific force and angular rate of the rig from the physics body and
        /// converts them to the streamer's convention. Unity is left-handed X-right/Y-up/Z-forward; the
        /// streamer expects right-handed X-left/Y-up/Z-forward (Isaac's FLU→optical change of basis
        /// followed by its 180° twist about Z). That is a mirror of the X axis, applied to both the
        /// world and the body frame: q → (w, x, -y, -z), vectors → (-x, y, z), pseudo-vectors → (x, -y, -z).
        /// </summary>
        private void SampleImu() {
            Quaternion rot = rigOrigin.rotation;
            Vector3 accelWorld = Vector3.zero;
            Vector3 angVelWorld = Vector3.zero;

            if (body != null) {
                Vector3 v = body.linearVelocity;
                if (velocityInitialised) accelWorld = (v - prevVelocity) / Time.fixedDeltaTime;
                prevVelocity = v;
                velocityInitialised = true;
                angVelWorld = body.angularVelocity;
            }

            Quaternion worldToRig = Quaternion.Inverse(rot);
            Vector3 specificForce = worldToRig * (accelWorld - Physics.gravity); // at rest: (0, +g, 0)
            Vector3 angVel = worldToRig * angVelWorld;

            imuOrientation = new Quaternion(rot.x, -rot.y, -rot.z, rot.w);
            imuSpecificForce = new Vector3(-specificForce.x, specificForce.y, specificForce.z);
            imuAngularVelocity = new Vector3(angVel.x, -angVel.y, -angVel.z);
        }

        private long CurrentTimestampNs() {
            double now = Time.timeAsDouble;
            if (!timeBaseSet) {
                epochBaseNs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() * 1_000_000L;
                simTimeBase = now;
                timeBaseSet = true;
            }
            return epochBaseNs + (long)((now - simTimeBase) * 1e9);
        }

        private void Update() {
            if (!streamerReady) return;
            double now = Time.timeAsDouble;

            if (logGroundTruthDepth && now - lastGroundTruthTime >= groundTruthInterval) {
                lastGroundTruthTime = now;
                LogGroundTruthDepth();
            }

            if (now - lastCaptureTime < 1.0 / fps) return;
            lastCaptureTime = now;

            FrameSlot slot;
            lock (sync) {
                if (freeSlots.Count == 0) return; // readbacks or the encoder are behind; drop this frame
                slot = freeSlots.Pop();
            }

            slot.timestampNs = CurrentTimestampNs();
            slot.orientation = imuOrientation;
            slot.specificForce = imuSpecificForce;
            slot.pendingReadbacks = 2;
            slot.failed = false;

            RequestEye(leftRT, leftFlipRT, slot, slot.left);
            RequestEye(rightRT, rightFlipRT, slot, slot.right);
        }

        private void RequestEye(RenderTexture source, RenderTexture flipped, FrameSlot slot, byte[] destination) {
            RenderTexture readbackSource = source;
            if (flipVertically) {
                Graphics.Blit(source, flipped, new Vector2(1f, -1f), new Vector2(0f, 1f));
                readbackSource = flipped;
            }
            AsyncGPUReadback.Request(readbackSource, 0, TextureFormat.RGBA32, request => OnEyeReadback(request, slot, destination));
        }

        private void OnEyeReadback(AsyncGPUReadbackRequest request, FrameSlot slot, byte[] destination) {
            if (request.hasError) {
                slot.failed = true;
            } else {
                var data = request.GetData<byte>();
                if (data.Length == destination.Length) data.CopyTo(destination);
                else slot.failed = true;
            }

            if (--slot.pendingReadbacks > 0) return;

            lock (sync) {
                if (slot.failed || stopping) {
                    freeSlots.Push(slot);
                    return;
                }
                if (readySlot != null) freeSlots.Push(readySlot); // newest frame wins
                readySlot = slot;
                Monitor.Pulse(sync);
            }
        }

        private void WorkerLoop() {
            while (true) {
                FrameSlot slot;
                lock (sync) {
                    while (readySlot == null && !stopping) Monitor.Wait(sync);
                    if (stopping) return;
                    slot = readySlot;
                    readySlot = null;
                }

                int result = ZedSimNative.StreamRgb(StreamerId,
                    slot.leftHandle.AddrOfPinnedObject(), slot.rightHandle.AddrOfPinnedObject(),
                    slot.timestampNs,
                    slot.orientation.w, slot.orientation.x, slot.orientation.y, slot.orientation.z,
                    slot.specificForce.x, slot.specificForce.y, slot.specificForce.z);
                if (result != 0) Debug.LogWarning($"ZedSimCamera: stream_rgb returned {result}");

                lock (sync) freeSlots.Push(slot);
            }
        }

        private void OnDisable() {
            lock (sync) {
                stopping = true;
                Monitor.PulseAll(sync);
            }
            if (worker != null && worker.IsAlive) worker.Join(2000);
            worker = null;

            if (streamerReady) {
                // The editor never unloads native plugins, so an unclosed streamer id would poison the
                // next play session (init_streamer returns -1).
                ZedSimNative.CloseStreamer(StreamerId);
                streamerReady = false;
            }
        }

        private void OnDestroy() {
            foreach (var slot in slots) {
                if (slot == null) continue;
                if (slot.leftHandle.IsAllocated) slot.leftHandle.Free();
                if (slot.rightHandle.IsAllocated) slot.rightHandle.Free();
            }
            foreach (var rt in new[] { leftRT, rightRT, leftFlipRT, rightFlipRT }) {
                if (rt != null) rt.Release();
            }
        }
    }
}
