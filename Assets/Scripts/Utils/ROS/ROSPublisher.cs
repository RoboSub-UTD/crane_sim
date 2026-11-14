using System;
using System.Reflection;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

namespace Sim.Utils.ROS {
    [AddComponentMenu("")]
    public class ROSPublisher : MonoBehaviour {
        public string topicName { get; set; }
        public string frameId { get; set; }
        public float Hz;
        public bool manualPublish;

        private ROSConnection ros;
        private double timeSincePublish;

        private Func<object> createMessage;
        private Action<string, object> publishTyped;

        private static void PublishWrapper<T>(ROSConnection ros, string topic, object msg)
        where T : Unity.Robotics.ROSTCPConnector.MessageGeneration.Message {
            ros.Publish(topic, (T)msg);
        }

        private void OnEnable() { hideFlags = HideFlags.HideInInspector; }

        public void Initialize<T>(string topicName, string frameId, Func<T> createMessage, float Hz=10f, bool manualPublish=false)
        where T : Unity.Robotics.ROSTCPConnector.MessageGeneration.Message {
            this.topicName = topicName;
            this.frameId = frameId;
            this.Hz = Hz;
            this.manualPublish = manualPublish;

            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<T>(topicName);

            this.createMessage = () => createMessage();

            var wrapperMethod = typeof(ROSPublisher)
                .GetMethod(nameof(PublishWrapper), BindingFlags.Static | BindingFlags.NonPublic)
                .MakeGenericMethod(typeof(T));

            publishTyped = (Action<string, object>)
                Delegate.CreateDelegate(
                    typeof(Action<string, object>),
                    ros,
                    wrapperMethod
                );
        }

        private void FixedUpdate() {
            if (manualPublish) return;
            if (createMessage == null) return;

            // Carry the remainder over instead of resetting to 0, otherwise float rounding adds an extra
            // physics step per period (e.g. 10 Hz at 50 Hz physics became ~8.3 Hz).
            double period = 1.0 / Hz;
            timeSincePublish += Time.fixedDeltaTime;
            if (timeSincePublish >= period - 1e-6) {
                Publish();
                timeSincePublish -= period;
                // Don't burst-publish to catch up if the period is shorter than a physics step
                if (timeSincePublish >= period) timeSincePublish = 0.0;
            }
        }

        public void Publish() {
            publishTyped(topicName, createMessage());
        }

        // Stamps with the current sim time (the physics step's time when called from FixedUpdate),
        // which ROSClock has already published on /clock. Pass stampSeconds to stamp data that was
        // captured earlier, e.g. an async GPU readback.
        public HeaderMsg CreateHeader(double? stampSeconds = null) {
            return new HeaderMsg {
                frame_id = frameId,
                stamp = Clock.ToTimeMsg(stampSeconds ?? Clock.time)
            };
        }
    }
}
