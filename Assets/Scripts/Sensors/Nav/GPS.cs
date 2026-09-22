
using System;
using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Sim.Utils;
using Sim.Utils.ROS;

namespace Sim.Sensors.Nav {
    /// <summary>
    /// Publishes a NavSatFix for this transform. The Unity world origin is pinned to
    /// <see cref="originLatitude"/>/<see cref="originLongitude"/>/<see cref="originAltitude"/> and the
    /// ENU offset (north = Unity axis set by the project's GeometryCompass) is projected onto the
    /// WGS-84 ellipsoid with a local tangent-plane approximation (error ~3 cm at 1.4 km from the
    /// origin, far below real GPS noise).
    /// </summary>
    public class GPS : MonoBehaviour, IROSSensor<NavSatFixMsg> {
        [SerializeField] private string topicName = "gps/raw";
        [SerializeField] private string frameId = "gps_link";
        [SerializeField] private float Hz = 20.0f;

        [Header("Datum (Unity world origin)")]
        [Tooltip("Defaults to Nathan Benderson Park, Sarasota FL (RoboBoat).")]
        [SerializeField] private double originLatitude = 27.374136;
        [SerializeField] private double originLongitude = -82.452767;
        [SerializeField] private double originAltitude = 0.0;

        [Header("Noise (1-sigma, metres)")]
        [SerializeField, Min(0f)] private float horizontalStdDev = 0.0f;
        [SerializeField, Min(0f)] private float verticalStdDev = 0.0f;
        [Tooltip("Floor on the reported covariance so filters never see a zero variance.")]
        [SerializeField, Min(0f)] private float minReportedStdDev = 0.01f;

        public ROSPublisher publisher { get; set; }

        // WGS-84
        private const double SemiMajorAxis = 6378137.0;
        private const double EccentricitySq = 6.69437999014e-3;

        private System.Random rng;

        public NavSatFixMsg CreateMessage() {
            Vector3<ENU> enu = transform.position.To<ENU>();
            double east = enu.x + Gaussian(horizontalStdDev);
            double north = enu.y + Gaussian(horizontalStdDev);
            double up = enu.z + Gaussian(verticalStdDev);

            double lat0 = originLatitude * Math.PI / 180.0;
            double sinLat = Math.Sin(lat0);
            double w = Math.Sqrt(1.0 - EccentricitySq * sinLat * sinLat);
            double primeVerticalRadius = SemiMajorAxis / w;
            double meridianRadius = SemiMajorAxis * (1.0 - EccentricitySq) / (w * w * w);

            double hVar = Math.Pow(Math.Max(horizontalStdDev, minReportedStdDev), 2);
            double vVar = Math.Pow(Math.Max(verticalStdDev, minReportedStdDev), 2);

            return new NavSatFixMsg {
                header = publisher.CreateHeader(),
                status = new NavSatStatusMsg {
                    status = NavSatStatusMsg.STATUS_FIX,
                    service = NavSatStatusMsg.SERVICE_GPS
                },
                latitude = originLatitude + north / meridianRadius * 180.0 / Math.PI,
                longitude = originLongitude + east / (primeVerticalRadius * Math.Cos(lat0)) * 180.0 / Math.PI,
                altitude = originAltitude + up,
                position_covariance = new[] { hVar, 0, 0, 0, hVar, 0, 0, 0, vVar },
                position_covariance_type = NavSatFixMsg.COVARIANCE_TYPE_DIAGONAL_KNOWN
            };
        }

        // Box-Muller
        private double Gaussian(float stdDev) {
            if (stdDev <= 0f) return 0.0;
            double u1 = 1.0 - rng.NextDouble();
            double u2 = rng.NextDouble();
            return stdDev * Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
        }

        private void Awake() {
            publisher = gameObject.AddComponent<ROSPublisher>();
            rng = new System.Random();
        }

        void Start() {
            publisher.Initialize(topicName, frameId, CreateMessage, Hz);
        }
    }
}
