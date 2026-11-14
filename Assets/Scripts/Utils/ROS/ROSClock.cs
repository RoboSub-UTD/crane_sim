using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Rosgraph;

namespace Sim.Utils.ROS {
    // Runs before every other script so that, within each physics step and each frame, /clock is
    // published before any sensor stamps a message with that same sim time.
    [DefaultExecutionOrder(-1000)]
    public class ROSClock : MonoBehaviour {
        [SerializeField] private Clock.ClockMode clockMode;

        [SerializeField, HideInInspector] private Clock.ClockMode lastSetClockMode;

        private double lastPublishTimeSeconds = double.NegativeInfinity;

        ROSConnection ros;

        private void OnValidate() {
            var clocks = FindObjectsByType<ROSClock>(FindObjectsSortMode.None);
            if (clocks.Length > 1) {
                Debug.LogWarning("Found too many clock publishers in the scene, there should only be one!");
            }

            if (Application.isPlaying && lastSetClockMode != clockMode) {
                Debug.LogWarning("Can't change ClockMode during simulation! Setting it back...");
                clockMode = lastSetClockMode;
            }

            SetClockMode(clockMode);
        }

        private void SetClockMode(Clock.ClockMode mode) {
            Clock.Mode = mode;
            lastSetClockMode = mode;
        }

        private void Awake() {
            SetClockMode(clockMode);
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<ClockMsg>("clock");
        }

        // Publish whenever sim time has advanced. Called at the start of every physics step (fixedTime)
        // and every frame (time); the monotonic guard keeps /clock from ever stepping backwards.
        private void PublishIfAdvanced() {
            var publishTime = Clock.time;
            if (publishTime <= lastPublishTimeSeconds) return;
            lastPublishTimeSeconds = publishTime;
            ros.Publish("clock", new ClockMsg(Clock.ToTimeMsg(publishTime)));
        }

        private void FixedUpdate() => PublishIfAdvanced();

        private void Update() => PublishIfAdvanced();
    }
}
