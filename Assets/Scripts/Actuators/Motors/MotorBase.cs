using UnityEngine;
using Sim.Utils;
using Sim.Utils.ROS;
using System;

namespace Sim.Actuators.Motors {
    public enum MotorControlMode {
        Torque,
        Velocity,
        Position
    }

    public enum Axis {
        X,
        Y,
        Z
    }

    public abstract class MotorBase<TConfig> : MonoBehaviour where TConfig : IMotorConfig, new() {
        [SerializeReference] protected TConfig config;
        [SerializeField] protected bool useDebugCommand;
        [SerializeField] protected float debugCommand;
        protected GameObject motorJoint;
        protected float maxTorque, maxAngularVelocity, torqueK, minAngle, maxAngle, maxAngularAcceleration, inertiaAlongAxis;
        protected MotorControlMode controlMode;
        protected Axis rotationAxis;
        protected Pid pid;
        protected Vector3 worldAxis, localAxis;
        protected IPhysicsBody body;
        protected float command;
        protected float torqueOutput;

        protected virtual void SetMotorDefaults() {
            motorJoint = gameObject;

            controlMode = config.controlMode;
            rotationAxis = config.rotationAxis;
            pid = config.pid?.Clone();
            maxTorque = config.maxTorque;
            torqueK = config.torqueK;
            maxAngularVelocity = config.maxAngularVelocity;
            minAngle = config.minAngle;
            maxAngle = config.maxAngle;
        }

        protected virtual void OnValidate() {
            if (GetComponent<Rigidbody>() == null && GetComponent<ArticulationBody>() == null)
                Debug.LogWarning($"{name} should have either a Rigidbody or an ArticulationBody attached.");

            if (minAngle > maxAngle) {
                (maxAngle, minAngle) = (minAngle, maxAngle);
                Debug.LogWarning($"{name}: Swapped min/max angles because min > max");
            }

            if (config == null) {
                if (typeof(ScriptableObject).IsAssignableFrom(typeof(TConfig)))
                    config = (TConfig)(object)ScriptableObject.CreateInstance(typeof(TConfig));
                else
                    config = Activator.CreateInstance<TConfig>();
            }

            SetMotorDefaults();
        }

        protected virtual void Awake() {
            var rb = GetComponent<Rigidbody>();
            var ab = GetComponent<ArticulationBody>();

            if (rb != null) body = new RigidbodyAdapter(rb);
            else if (ab != null) body = new ArticulationBodyAdapter(ab);
            else throw new MissingComponentException($"{name} requires a Rigidbody or ArticulationBody!");
        }

        protected virtual void Start() {
            if (motorJoint == null)
                throw new MissingReferenceException($"{name}: motorJoint is not assigned.");

            localAxis = rotationAxis switch {
                Axis.X => Vector3.right,
                Axis.Y => Vector3.up,
                Axis.Z => Vector3.forward,
                _ => Vector3.up
            };

            inertiaAlongAxis = Vector3.Dot(localAxis, body.inertiaTensorRotation * Vector3.Scale(body.inertiaTensor, localAxis));
            maxAngularAcceleration = maxTorque / inertiaAlongAxis;
            body.maxAngularVelocity = maxAngularVelocity;

            torqueOutput = 0;
        }

        protected virtual void FixedUpdate() {
            command = useDebugCommand ? debugCommand : command;
            worldAxis = motorJoint.transform.TransformDirection(localAxis);

            switch (controlMode) {
                case MotorControlMode.Torque:
                    ApplyTorqueControl(command);
                    break;

                case MotorControlMode.Velocity:
                    ApplyVelocityControl(command);
                    break;

                case MotorControlMode.Position:
                    ApplyPositionControl(command);
                    break;
            }
        }

        protected virtual void ApplyTorqueControl(float torque) {
            // Smooth torque application using torqueK to simplify external factors slowing torque (near instant)
            torque = Mathf.Clamp(torque, -maxTorque, maxTorque);

            if (controlMode != MotorControlMode.Position)
                IsAtLimit(ref torque);

            torqueOutput = Mathf.Lerp(torqueOutput, torque, 1 - Mathf.Pow(1 - torqueK, Time.deltaTime * 60f));

            body.AddTorque(worldAxis * torqueOutput);
        }

        protected virtual void ApplyVelocityControl(float targetVelocity) {
            float torque = inertiaAlongAxis * (targetVelocity - GetVelocity()) / Time.fixedDeltaTime;

            if (controlMode != MotorControlMode.Position)
                IsAtLimit(ref targetVelocity);

            ApplyTorqueControl(torque);
        }

        // Uses Pid controller for manual control (would have to be implemented in our code normally)
        protected virtual void ApplyPositionControl(float targetAngle) {
            if (pid == null) return;

            targetAngle = Mathf.Clamp(targetAngle, minAngle, maxAngle);
            float currentAngle = GetAngle();
            float angleError = Mathf.DeltaAngle(currentAngle, targetAngle);
            float desiredVelocity = Mathf.Deg2Rad * pid.Run(angleError, Time.fixedDeltaTime);
            desiredVelocity = Mathf.Clamp(desiredVelocity, -maxAngularVelocity, maxAngularVelocity);

            ApplyVelocityControl(desiredVelocity);
        }

        protected virtual float GetAlongAxis(Vector3 v, Axis axis) =>
            axis switch {
                Axis.X => v.x,
                Axis.Y => v.y,
                Axis.Z => v.z,
                _ => v.y
            };

        public virtual float GetVelocity() => GetAlongAxis(body.transform.InverseTransformDirection(body.angularVelocity), rotationAxis);
        public virtual float GetAngle() => GetAlongAxis(motorJoint.transform.localEulerAngles, rotationAxis);
        public virtual float GetCommand() => command;
        public virtual float SetCommand(float command) => this.command = command;
        public virtual TConfig GetConfig() => config;
        public virtual float GetMaxCommand() => config.controlMode switch {
            MotorControlMode.Torque => config.maxTorque,
            MotorControlMode.Velocity => config.maxAngularVelocity,
            MotorControlMode.Position => config.maxAngle,
            _ => 0.0f
        };
        public virtual float GetMinCommand() => config.controlMode switch {
            MotorControlMode.Torque => -config.maxTorque,
            MotorControlMode.Velocity => -config.maxAngularVelocity,
            MotorControlMode.Position => config.minAngle,
            _ => 0.0f
        };

        protected virtual bool IsAtLimit(ref float command) {
            float currentAngle = GetAngle();
            if ((currentAngle <= minAngle && command < 0f) ||
                (currentAngle >= maxAngle && command > 0f)) {
                command = 0f;
                body.angularVelocity = Vector3.zero;
                return true;
            }
            return false;
        }
    }
}
