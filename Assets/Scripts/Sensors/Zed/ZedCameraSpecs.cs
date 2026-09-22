using UnityEngine;

namespace Sim.Sensors.Zed {
    public enum ZedResolution { HD2K, HD1080, HD720, VGA }
    public enum ZedLens { Wide, Narrow }

    /// <summary>
    /// Optical parameters of the virtual ZED 2i that the ZED SDK's built-in simulation calibration
    /// assumes (values mirror the Isaac Sim extension's camera table). The rendered images must match
    /// these exactly, otherwise the SDK's rectification/depth is wrong.
    /// </summary>
    public static class ZedCameraSpecs {
        /// <summary>sl::MODEL code the SDK's virtual serial pool uses for the ZED 2i.</summary>
        public const int Zed2iModelId = 3;

        /// <summary>Distance between the two optical centres, metres.</summary>
        public const float Baseline = 0.12f;

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
