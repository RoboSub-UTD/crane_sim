using System;
using UnityEngine;
using Sim.Utils;
using UnityEngine.Rendering.HighDefinition;
using Sim.Physics.Processing;

namespace Sim.Actuators.Motors {
    public class Thruster : MotorBase<ThrusterConfig> {
        [SerializeField] private WaterSurface waterSurface; // Can't put GameObjects into ScriptableObjects
        [SerializeField] private bool debug;
        private IPhysicsBody rootBody;
        private float thrustK, backK, height;

        protected override void Awake() {
            base.Awake();

            thrustK = config.thrustK;
            backK = config.backK;
            height = config.height;

            var parentRb = transform.root.GetComponent<Rigidbody>();
            var parentAb = transform.root.GetComponent<ArticulationBody>();

            if (parentRb != null) rootBody = new RigidbodyAdapter(parentRb);
            else if (parentAb != null) rootBody = new ArticulationBodyAdapter(parentAb);
            else rootBody = body;
        }

        protected override void FixedUpdate() {
            base.FixedUpdate();

            float waterHeight = WaterUtils.Search(waterSurface, body.transform.position).projectedPositionWS.y;
            float depth = waterHeight - body.transform.position.y; // positive if below water
            float submersionFraction = Mathf.Clamp01(depth / height);

            float velocity = base.GetVelocity();
            Vector3 forceDirection = rootBody.transform.TransformDirection(transform.localRotation * localAxis);
            Vector3 force = velocity * thrustK * forceDirection;
            force *= Mathf.Sign(velocity) < 0 ? backK : 1;
            force *= submersionFraction;
            rootBody.AddForceAtPosition(force, body.transform.position);

            if (debug) Debug.DrawLine(body.position, body.position - force * 0.01f, Color.red, Time.fixedDeltaTime);
        }

        public float GetMaxThrust() => config.GetMaxThrust();
        public float GetMaxBackThrust() => config.GetMaxBackThrust();
    }
}
