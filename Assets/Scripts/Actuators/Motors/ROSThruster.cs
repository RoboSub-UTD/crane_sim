using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.GalaxseaInterfaces;

namespace Sim.Actuators.Motors {
    public class ROSThruster : Thruster {
        public enum ThrusterPosition {
            FrontLeft,
            FrontRight,
            BackLeft,
            BackRight
        }

        [SerializeField] private string topicName = "/thruster_commands";
        [SerializeField] private ThrusterPosition position;

        private ROSConnection ros;

        protected override void Awake() {
            base.Awake();

            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<ThrusterCommandMsg>(topicName, CommandCallback);
        }

        private void CommandCallback(ThrusterCommandMsg msg) {
            float command = position switch {
                ThrusterPosition.FrontLeft => (float)msg.front_left,
                ThrusterPosition.FrontRight => (float)msg.front_right,
                ThrusterPosition.BackLeft => (float)msg.back_left,
                ThrusterPosition.BackRight => (float)msg.back_right,
                _ => 0f
            };

            base.SetCommand(command);
        }
    }
}
