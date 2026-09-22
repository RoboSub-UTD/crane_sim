using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Sim.Sensors.Zed {
    /// <summary>
    /// P/Invoke binding to the simulation-streamer entry points exported by Stereolabs'
    /// <c>libsl_zed</c> (the same functions the official Isaac Sim extension calls).
    /// The library is not committed; install it with <c>Tools ▸ ZED Sim ▸ Install streamer library</c>.
    /// </summary>
    public static class ZedSimNative {
#if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
        private const string Lib = "sl_zed64";
#else
        private const string Lib = "sl_zed";
#endif

        public enum InputFormat : int { RGB = 0, BGR = 1, YUV = 2 }
        public enum TransportLayer : int { Network = 0, Ipc = 1, Both = 2 }
        public enum SimLensType : int { Wide = 0, Narrow = 1, Fisheye = 2 }

        /// <summary>Mirror of <c>sl::StreamingParameters</c> (types_c.h). Field order and widths must not change.</summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct StreamingParameters {
            public int mode;                    // streaming protocol generation (1)
            public float imuCamQx, imuCamQy, imuCamQz, imuCamQw; // imu -> camera quaternion, IMAGE convention
            public float imuCamTx, imuCamTy, imuCamTz;           // imu -> camera translation
            public int imageWidth;
            public int imageHeight;
            public int codecType;               // 0 = H264, 1 = H265
            public ushort port;
            public int fps;                     // frames above this rate are dropped by the streamer
            public int serialNumber;            // must come from get_virtual_camera_info
            public byte alphaChannelIncluded;   // bool
            public InputFormat inputFormat;
            public byte verbose;                // bool
            public TransportLayer transportLayerMode;
            public int bitrateKbps;
            public ushort chunkSize;
            public int gopSize;                 // -1 = codec default
            public byte gpuInput;               // bool: buffers are CUDA device pointers
            public byte streamDepth;            // bool: left + float depth instead of left + right
            public int depthWidth;
            public int depthHeight;
            public int depthBitrate;

            public static StreamingParameters Default() => new StreamingParameters {
                mode = 1,
                imuCamQx = 0, imuCamQy = 0, imuCamQz = 0, imuCamQw = 1,
                imageWidth = 1920,
                imageHeight = 1200,
                codecType = 1,
                port = 30000,
                fps = 30,
                serialNumber = 0,
                alphaChannelIncluded = 1,
                inputFormat = InputFormat.BGR,
                verbose = 0,
                transportLayerMode = TransportLayer.Network,
                bitrateKbps = 8000,
                chunkSize = 4096,
                gopSize = -1,
            };
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct SimCameraInfo {
            public int serialNumber;
            public int model;      // sl::MODEL code (3 = ZED 2i)
            public SimLensType lensType;
        }

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "getZEDSDKRuntimeVersion_C")]
        private static extern int GetRuntimeVersionNative(ref int major, ref int minor, ref int patch);

        /// <returns>1 on success, 0 if the RTP session could not be created, -1 if the id is already in use.</returns>
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "init_streamer")]
        public static extern int InitStreamer(int streamerId, ref StreamingParameters parameters);

        /// <summary>Push one stereo frame. Buffers are tightly packed width*height*4 BGRA8 (or RGBA8), top row first.</summary>
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "stream_rgb")]
        public static extern int StreamRgb(int streamerId, IntPtr left, IntPtr right, long timestampNs,
            float qw, float qx, float qy, float qz, float accX, float accY, float accZ);

        /// <summary>Push a left image plus a float32 depth map (metres) instead of a stereo pair.</summary>
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "stream_left_and_depth")]
        public static extern int StreamLeftAndDepth(int streamerId, IntPtr left, IntPtr depth, long timestampNs,
            float qw, float qx, float qy, float qz, float accX, float accY, float accZ);

        /// <summary>Optional high-rate IMU sample (gyro rad/s, specific force m/s², orientation).</summary>
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "ingest_imu")]
        public static extern int IngestImu(int streamerId, long timestampNs,
            float gyroX, float gyroY, float gyroZ,
            float accX, float accY, float accZ,
            float qw, float qx, float qy, float qz);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "close_streamer")]
        public static extern void CloseStreamer(int streamerId);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "destroy_instance")]
        public static extern void DestroyInstance();

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "is_sn_valid")]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool IsSerialValid(int serialNumber);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "get_virtual_camera_info")]
        private static extern IntPtr GetVirtualCameraInfoNative(ref int count);

        /// <summary>Runtime version of the bundled library, or null if the library can't be loaded.</summary>
        public static Version TryGetRuntimeVersion() {
            try {
                int major = 0, minor = 0, patch = 0;
                if (GetRuntimeVersionNative(ref major, ref minor, ref patch) != 0) return null;
                return new Version(major, minor, patch);
            } catch (DllNotFoundException) {
                return null;
            } catch (EntryPointNotFoundException) {
                return null;
            }
        }

        /// <summary>All virtual cameras the SDK knows how to calibrate.</summary>
        public static List<SimCameraInfo> GetVirtualCameras() {
            var result = new List<SimCameraInfo>();
            int count = 0;
            IntPtr ptr = GetVirtualCameraInfoNative(ref count);
            if (ptr == IntPtr.Zero) return result;

            int stride = Marshal.SizeOf<SimCameraInfo>();
            for (int i = 0; i < count; i++)
                result.Add(Marshal.PtrToStructure<SimCameraInfo>(ptr + i * stride));
            return result;
        }

        /// <summary>Picks the first valid virtual serial for a model/lens pair, or 0 if none exists.</summary>
        public static int PickVirtualSerial(int modelId, SimLensType lens) {
            foreach (var info in GetVirtualCameras()) {
                if (info.model == modelId && info.lensType == lens && IsSerialValid(info.serialNumber))
                    return info.serialNumber;
            }
            return 0;
        }

        public static string DescribeInitResult(int code) => code switch {
            1 => "ok",
            0 => "RTP session creation failed (port already bound? NVENC unavailable?)",
            -1 => "streamer id already in use (previous session not closed)",
            _ => $"unknown result {code}",
        };
    }
}
