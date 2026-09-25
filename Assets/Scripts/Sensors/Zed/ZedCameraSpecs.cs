using UnityEngine;

namespace Sim.Sensors.Zed {
    public enum ZedResolution { HD2K, HD1080, HD720, VGA }
    public enum ZedLens { Wide, Narrow }

    /// <summary>
    /// Optical parameters of the virtual ZED 2i that the ZED SDK's built-in simulation calibration
    /// assumes (values mirror the Isaac Sim extension's camera table). The rendered images must match
    /// these exactly, otherwise the SDK's rectification/depth is wrong.
    ///
    /// The SDK derives depth from disparity using the calibration it attaches to the virtual serial,
    /// not from anything the streamer sends, so depth is metric only while the rendered geometry
    /// agrees with it. Read back from a running wrapper (v5.4.1, HD720, serial 20976320):
    /// fx = fy = 529.8, cx = 640, cy = 360, zero distortion, and P[3] = -63.576 on the right camera,
    /// i.e. a baseline of 63.576 / 529.8 = 120.000 mm. <see cref="ZedSimCamera"/> re-checks the rig
    /// against these numbers at start-up.
    /// </summary>
    public static class ZedCameraSpecs {
        /// <summary>sl::MODEL code the SDK's virtual serial pool uses for the ZED 2i.</summary>
        public const int Zed2iModelId = 3;

        /// <summary>Distance between the two optical centres, metres.</summary>
        public const float Baseline = 0.12f;

        /// <summary>
        /// How far the optical centres sit behind the centre of the camera body, metres.
        /// zed-ros2-wrapper's zed_macro.urdf.xacro puts the ZED 2i's left/right camera frames at
        /// (optical_offset_x, ±baseline/2, 0) = (-0.01, ±0.06, 0) from zed_camera_center; mirroring
        /// that keeps the simulated cloud in the same place relative to the camera body as on the
        /// real camera, so the wrapper's static transforms stay true in simulation.
        /// </summary>
        public const float OpticalCentreOffset = 0.01f;

        /// <summary>
        /// Height of zed_camera_center above zed_camera_link (the tripod screw on the bottom of the
        /// body), metres: the xacro's (0, 0, height/2) with the ZED 2i's height 0.03 and zero
        /// bottom_slope. zed_camera_link is the root of the wrapper's static TF tree, so it is the
        /// frame the robot's tree has to attach it by.
        /// </summary>
        public const float MountToCentreHeight = 0.015f;

        /// <summary>Left optical centre relative to the body centre, in eye axes (X right, Y up, Z forward).</summary>
        public static Vector3 LeftEyeOffset => new Vector3(-Baseline / 2f, 0f, -OpticalCentreOffset);

        /// <summary>Right optical centre relative to the body centre, in eye axes.</summary>
        public static Vector3 RightEyeOffset => new Vector3(+Baseline / 2f, 0f, -OpticalCentreOffset);

        public struct Spec {
            public int width;
            public int height;
            public float fx; // rectified focal length in pixels, also fy

            public float VerticalFov => 2f * Mathf.Atan(height / (2f * fx)) * Mathf.Rad2Deg;
            public float HorizontalFov => 2f * Mathf.Atan(width / (2f * fx)) * Mathf.Rad2Deg;
        }

        public static Spec Get(ZedResolution resolution, ZedLens lens) {
            // Wide = 2.1 mm lens; narrow = 4 mm lens scales fx by 4.0 / 2.1.
            Spec s = resolution switch {
                ZedResolution.HD2K => new Spec { width = 2208, height = 1242, fx = 1059.6f },
                ZedResolution.HD1080 => new Spec { width = 1920, height = 1080, fx = 1059.6f },
                ZedResolution.HD720 => new Spec { width = 1280, height = 720, fx = 529.8f },
                _ => new Spec { width = 672, height = 376, fx = 264.9f },
            };
            if (lens == ZedLens.Narrow) s.fx *= 4.0f / 2.1f;
            return s;
        }

        public static ZedSimNative.SimLensType ToSimLens(ZedLens lens)
            => lens == ZedLens.Narrow ? ZedSimNative.SimLensType.Narrow : ZedSimNative.SimLensType.Wide;

        /// <summary>Reasonable H.265 bitrate for the resolution, kbps.</summary>
        public static int DefaultBitrateKbps(ZedResolution resolution) => resolution switch {
            ZedResolution.HD2K => 12000,
            ZedResolution.HD1080 => 8000,
            ZedResolution.HD720 => 4000,
            _ => 2000,
        };
    }
}
