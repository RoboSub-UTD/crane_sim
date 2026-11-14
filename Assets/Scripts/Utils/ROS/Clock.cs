using System;
using UnityEngine;
using RosMessageTypes.BuiltinInterfaces;

namespace Sim.Utils.ROS {
    public static class Clock {
        // Since UnityScaled is the default Unity Time mode, we'll use that for this project
        // None of the other time modes are fully validated and guaranteed to be without issues
        public enum ClockMode {
            // Time delta scaled by simulation speed to the start of the current frame, since the beginning of the game.
            // Inside FixedUpdate this is the start of the current physics step (Time.fixedTimeAsDouble).
            UnityScaled,
            // Examples of other potentially useful clock modes...
            // UnityUnscaled, UnixEpoch, UnixEpochUtc, DateTimeNow, DateTimeNowUtc, ExternalClock
        }

        public const double k_NanoSecondsInSeconds = 1e9;

        // Sim time of the current frame, or of the current physics step when called from FixedUpdate.
        // This is the only time base used for both /clock and message header stamps, so a stamp can
        // never be ahead of the /clock that ROSClock has already published (ROSClock runs first).
        public static double FrameStartTimeInSeconds {
            get {
                return Mode switch {
                    ClockMode.UnityScaled => Time.timeAsDouble,
                    _ => throw new NotImplementedException()
                };
            }
        }

        // NOTE: Precision loss vs. other time measurements due to no deltaTimeAsDouble interface
        public static float DeltaTimeInSeconds {
            get {
                return Mode switch {
                    ClockMode.UnityScaled => Time.deltaTime,
                    _ => Time.unscaledDeltaTime,
                };
            }
        }

        public static ClockMode Mode = ClockMode.UnityScaled;

        // Simple interfaces for supporting commonly used vocabulary
        public static double time => FrameStartTimeInSeconds;
        public static float deltaTime => DeltaTimeInSeconds;

        public static TimeMsg ToTimeMsg(double seconds) {
            double whole = Math.Floor(seconds);
            return new TimeMsg {
                sec = (int)whole,
                nanosec = (uint)((seconds - whole) * k_NanoSecondsInSeconds)
            };
        }
    }
}
