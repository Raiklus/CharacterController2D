using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CooldHands
{
    /// <summary>
    /// Represents the entire state of a PhysicsMover that is pertinent for simulation.
    /// Use this to save state or revert to past state
    /// </summary>
    [System.Serializable]
    public struct PhysicsMoverState
    {
        public Vector2 Position;
        public Quaternion Rotation;
        public Vector2 Velocity;
        public float AngularVelocity;
    }

    /// <summary>
    /// Component that manages the movement of moving kinematic rigidbodies for
    /// proper interaction with characters
    /// </summary>
    [RequireComponent(typeof(Rigidbody2D))]
    public class PhysicsMover : MonoBehaviour
    {
        /// <summary>
        /// The mover's Rigidbody
        /// </summary>
        [ReadOnly]
        public Rigidbody2D Rigidbody;
        /// <summary>
        /// The BaseMoverController that manages this mover
        /// </summary>
        public BaseMoverController MoverController;

        /// <summary>
        /// Index of this motor in KinematicCharacterSystem arrays
        /// </summary>
        public int IndexInCharacterSystem { get; set; }
        /// <summary>
        /// Remembers initial position before all simulation are done
        /// </summary>
        public Vector2 InitialTickPosition { get; set; }
        /// <summary>
        /// Remembers initial rotation before all simulation are done
        /// </summary>
        public Quaternion InitialTickRotation { get; set; }

        /// <summary>
        /// The mover's Transform
        /// </summary>
        public Transform Transform { get; private set; }
        /// <summary>
        /// The character's position before the movement calculations began
        /// </summary>
        public Vector2 InitialSimulationPosition { get; private set; }
        /// <summary>
        /// The character's rotation before the movement calculations began
        /// </summary>
        public Quaternion InitialSimulationRotation { get; private set; }

        private Vector2 _internalTransientPosition;
        /// <summary>
        /// The mover's rotation (always up-to-date during the character update phase)
        /// </summary>
        public Vector2 TransientPosition
        {
            get
            {
                return _internalTransientPosition;
            }
            private set
            {
                _internalTransientPosition = value;
            }
        }

        private Quaternion _internalTransientRotation;
        /// <summary>
        /// The mover's rotation (always up-to-date during the character update phase)
        /// </summary>
        public Quaternion TransientRotation
        {
            get
            {
                return _internalTransientRotation;
            }
            private set
            {
                _internalTransientRotation = value;
            }
        }


        private void Reset()
        {
            ValidateData();
        }

        private void OnValidate()
        {
            ValidateData();
        }

        /// <summary>
        /// Handle validating all required values
        /// </summary>
        public void ValidateData()
        {
            Rigidbody = gameObject.GetComponent<Rigidbody2D>();

            Rigidbody.centerOfMass = Vector3.zero;
           // Rigidbody.useGravity = false;
            Rigidbody.drag = 0f;
            Rigidbody.angularDrag = 0f;
            //Rigidbody.maxAngularVelocity = Mathf.Infinity;
           // Rigidbody.maxDepenetrationVelocity = Mathf.Infinity;
            Rigidbody.collisionDetectionMode = CollisionDetectionMode2D.Discrete;
            Rigidbody.isKinematic = true;
            Rigidbody.constraints = RigidbodyConstraints2D.None;
			Rigidbody.interpolation = CharacterControllerManager.InterpolationMethod == CharacterManagerInterpolationMethod.Unity ? RigidbodyInterpolation2D.Interpolate : RigidbodyInterpolation2D.None;
        }

        private void OnEnable()
        {
			CharacterControllerManager.EnsureCreation();
			CharacterControllerManager.RegisterPhysicsMover(this);
        }

        private void OnDisable()
        {
			CharacterControllerManager.UnregisterPhysicsMover(this);
        }

        private void Awake()
        {
            Transform = this.transform;
            ValidateData();

            MoverController.SetupMover(this);

            TransientPosition = Rigidbody.position;
			TransientRotation = Quaternion.Euler(new Vector3(0,0, Rigidbody.rotation));
            InitialSimulationPosition = Rigidbody.position;
			InitialSimulationRotation =Quaternion.Euler(new Vector3(0,0, Rigidbody.rotation));
        }

        /// <summary>
        /// Sets the mover's position directly
        /// </summary>
        public void SetPosition(Vector3 position)
        {
            Rigidbody.interpolation = RigidbodyInterpolation2D.None;
            Transform.position = position;
            Rigidbody.position = position;
            InitialSimulationPosition = position;
            TransientPosition = position;
			Rigidbody.interpolation = CharacterControllerManager.InterpolationMethod == CharacterManagerInterpolationMethod.Unity ? RigidbodyInterpolation2D.Interpolate : RigidbodyInterpolation2D.None;
        }

        /// <summary>
        /// Sets the mover's rotation directly
        /// </summary>
        public void SetRotation(Quaternion rotation)
        {
			Rigidbody.interpolation = RigidbodyInterpolation2D.None;
            Transform.rotation = rotation;
            Rigidbody.rotation = rotation.z;
            InitialSimulationRotation = rotation;
            TransientRotation = rotation;
			Rigidbody.interpolation = CharacterControllerManager.InterpolationMethod == CharacterManagerInterpolationMethod.Unity ? RigidbodyInterpolation2D.Interpolate : RigidbodyInterpolation2D.None;
        }

        /// <summary>
        /// Sets the mover's position and rotation directly
        /// </summary>
        public void SetPositionAndRotation(Vector3 position, Quaternion rotation)
        {
			Rigidbody.interpolation = RigidbodyInterpolation2D.None;
            Transform.SetPositionAndRotation(position, rotation);
            Rigidbody.position = position;
            Rigidbody.rotation = rotation.z;
            InitialSimulationPosition = position;
            InitialSimulationRotation = rotation;
            TransientPosition = position;
            TransientRotation = rotation;
			Rigidbody.interpolation = CharacterControllerManager.InterpolationMethod == CharacterManagerInterpolationMethod.Unity ? RigidbodyInterpolation2D.Interpolate : RigidbodyInterpolation2D.None;
        }

        /// <summary>
        /// Returns all the state information of the mover that is pertinent for simulation
        /// </summary>
        public PhysicsMoverState GetState()
        {
            PhysicsMoverState state = new PhysicsMoverState();

            state.Position = TransientPosition;
            state.Rotation = TransientRotation;
            state.Velocity = Rigidbody.velocity;
            state.AngularVelocity = Rigidbody.angularVelocity;

            return state;
        }

        /// <summary>
        /// Applies a mover state instantly
        /// </summary>
        public void ApplyState(PhysicsMoverState state)
        {
            SetPositionAndRotation(state.Position, state.Rotation);
            Rigidbody.velocity = state.Velocity;
            Rigidbody.angularVelocity = state.AngularVelocity;
        }

        /// <summary>
        /// Caches velocity values based on deltatime and target position/rotations
        /// </summary>
        public void VelocityUpdate(float deltaTime)
        {
            InitialSimulationPosition = TransientPosition;
            InitialSimulationRotation = TransientRotation;

            MoverController.UpdateMovement(ref _internalTransientPosition, ref _internalTransientRotation, deltaTime);

            if (deltaTime > 0f)
            {
                Rigidbody.velocity = (TransientPosition - InitialSimulationPosition) / deltaTime;
                                
                Quaternion rotationFromCurrentToGoal = TransientRotation * (Quaternion.Inverse(InitialSimulationRotation));
				Rigidbody.angularVelocity = ((Mathf.Deg2Rad * rotationFromCurrentToGoal.eulerAngles) / deltaTime).z;
            }
        }
    }
}