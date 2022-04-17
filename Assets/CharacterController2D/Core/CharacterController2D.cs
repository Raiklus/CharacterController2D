using UnityEngine;
using System.Collections;

namespace CooldHands
{
    public enum RigidbodyInteractionType
    {
        None,
        Kinematic,
        SimulatedDynamic
    }

    public enum StepHandlingMethod
    {
        None,
        Standard,
        Extra
    }

    public enum MovementSweepState
    {
        Initial,
        AfterFirstHit,
        FoundBlockingCrease,
        FoundBlockingCorner,
    }

    /// <summary>
    /// Represents the entire state of a character motor that is pertinent for simulation.
    /// Use this to save state or revert to past state
    /// </summary>
    [System.Serializable]
    public struct CharacterState
    {
        public Vector2 Position;
        public Quaternion Rotation;
        public Vector2 BaseVelocity;

        public bool MustUnground;
        public bool LastMovementIterationFoundAnyGround;
        public CharacterTransientGroundingReport GroundingStatus;

        public Rigidbody2D AttachedRigidbody; 
        public Vector2 AttachedRigidbodyVelocity; 
    }

    /// <summary>
    /// Describes an overlap between the character capsule and another collider
    /// </summary>
    public struct OverlapResult
    {
        public Vector2 Normal;
        public Collider2D Collider;

		public OverlapResult(Vector2 normal, Collider2D collider)
        {
            Normal = normal;
            Collider = collider;
        }
    }
		

	/// <summary>
	/// Contains all the information for hit status
	/// </summary>
	public struct CharacterHitReport
	{
		public bool FoundAnyHit;
		public Vector2 HitNormal;
		public Collider2D HitCollider;
		public Vector2 HitPoint;
	}

    /// <summary>
    /// Contains all the information for the motor's grounding status
    /// </summary>
    public struct CharacterGroundingReport
    {
        public bool FoundAnyGround;
        public bool IsStableOnGround;
        public bool SnappingPrevented;
        public Vector2 GroundNormal;
        public Vector2 InnerGroundNormal;
        public Vector2 OuterGroundNormal;

        public Collider2D GroundCollider;
        public Vector2 GroundPoint;

        public void CopyFrom(CharacterTransientGroundingReport transientGroundingReport)
        {
            FoundAnyGround = transientGroundingReport.FoundAnyGround;
            IsStableOnGround = transientGroundingReport.IsStableOnGround;
            SnappingPrevented = transientGroundingReport.SnappingPrevented;
            GroundNormal = transientGroundingReport.GroundNormal;
            InnerGroundNormal = transientGroundingReport.InnerGroundNormal;
            OuterGroundNormal = transientGroundingReport.OuterGroundNormal;

            GroundCollider = null;
            GroundPoint = Vector3.zero;
        }
    }

    /// <summary>
    /// Contains the simulation-relevant information for the motor's grounding status
    /// </summary>
    public struct CharacterTransientGroundingReport
    {
        public bool FoundAnyGround;
        public bool IsStableOnGround;
        public bool SnappingPrevented;
        public Vector2 GroundNormal; 
        public Vector2 InnerGroundNormal;
        public Vector2 OuterGroundNormal;

        public void CopyFrom(CharacterGroundingReport groundingReport)
        {
            FoundAnyGround = groundingReport.FoundAnyGround;
            IsStableOnGround = groundingReport.IsStableOnGround;
            SnappingPrevented = groundingReport.SnappingPrevented;
            GroundNormal = groundingReport.GroundNormal;
            InnerGroundNormal = groundingReport.InnerGroundNormal;
            OuterGroundNormal = groundingReport.OuterGroundNormal;
        }
    }

    /// <summary>
    /// Contains all the information from a hit stability evaluation
    /// </summary>
    public struct HitStabilityReport
    {
        public bool IsStable;

        public Vector2 InnerNormal;
        public Vector2 OuterNormal;

        public bool ValidStepDetected;
        public Collider2D SteppedCollider;

        public bool LedgeDetected;
        public bool IsOnEmptySideOfLedge;
        public float DistanceFromLedge;
        public Vector2 LedgeGroundNormal;
        public Vector2 LedgeRightDirection;
        public Vector2 LedgeFacingDirection;
    }

    /// <summary>
    /// Contains the information of hit rigidbodies during the movement phase, so they can be processed afterwards
    /// </summary>
    public struct RigidbodyProjectionHit
    {
        public Rigidbody2D Rigidbody;
        public Vector2 HitPoint;
        public Vector2 EffectiveHitNormal;
        public Vector2 HitVelocity;
        public bool StableOnHit;
    }
    
    /// <summary>
    /// Component that manages character collisions and movement solving
    /// </summary>
    [RequireComponent(typeof(Rigidbody2D))]
    [RequireComponent(typeof(CapsuleCollider2D))]
    public class CharacterController2D : MonoBehaviour
    {
#pragma warning disable 0414
        /// <summary>
        /// The BaseCharacterController that manages this motor
        /// </summary>
        [Header("Components")]
        public BaseCharacterController CharacterController;
        /// <summary>
        /// The capsule collider of this motor
        /// </summary>
        [ReadOnly]
        public CapsuleCollider2D Capsule;
        /// <summary>
        /// The rigidbody of this motor
        /// </summary>
        [ReadOnly]
        public Rigidbody2D Rigidbody;

        [Header("Capsule Settings")]
        /// <summary>
        /// Radius of the character's capsule
        /// </summary>
        [SerializeField]
        [Tooltip("Width of the Character Capsule")]
        private float CapsuleWidth = 0.5f;
        /// <summary>
        /// Height of the character's capsule
        /// </summary>
        [SerializeField]
        [Tooltip("Height of the Character Capsule")]
        private float CapsuleHeight = 2f;
        /// <summary>
        /// Local y position of the character's capsule center
        /// </summary>
        [SerializeField]
        [Tooltip("Height of the Character Capsule")]
        private float CapsuleYOffset = 1f;
        /// <summary>
        /// Physics material of the character's capsule
        /// </summary>
        [SerializeField]
        [Tooltip("Physics material of the Character Capsule (Does not affect character movement. Only affects things colliding with it)")]
        private PhysicsMaterial2D CapsulePhysicsMaterial;

        [Header("Misc Options")]

        /// <summary>
        /// Notifies the Character Controller when discrete collisions are detected
        /// </summary>    
        [Tooltip("Notifies the Character Controller when discrete collisions are detected")]
        public bool DetectDiscreteCollisions = false;
        /// <summary>
        /// Increases the range of ground detection, to allow snapping to ground at very high speeds
        /// </summary>    
        [Tooltip("Increases the range of ground detection, to allow snapping to ground at very high speeds")]
        public float GroundDetectionExtraDistance = 0f;
        /// <summary>
        /// Maximum height of a step which the character can climb
        /// </summary>    
        [Tooltip("Maximum height of a step which the character can climb")]
        public float MaxStepHeight = 0.5f;
        /// <summary>
        /// Minimum length of a step that the character can step on (used in Extra stepping method. Use this to let the character step on steps that are smaller that its radius
        /// </summary>    
        [Tooltip("Minimum length of a step that the character can step on (used in Extra stepping method). Use this to let the character step on steps that are smaller that its radius")]
        public float MinRequiredStepDepth = 0.1f;
        /// <summary>
        /// Maximum slope angle on which the character can be stable
        /// </summary>    
        [Range(0f, 89f)]
        [Tooltip("Maximum slope angle on which the character can be stable")]
        public float MaxStableSlopeAngle = 60f;
        /// <summary>
        /// The distance from the capsule central axis at which the character can stand on a ledge and still be stable
        /// </summary>    
        [Tooltip("The distance from the capsule central axis at which the character can stand on a ledge and still be stable")]
        public float MaxStableDistanceFromLedge = 0.5f;
        /// <summary>
        /// Prevents snapping to ground on ledges. Set this to true if you want more determinism when launching off slopes
        /// </summary>    
        [Tooltip("Prevents snapping to ground on ledges. Set this to true if you want more determinism when launching off slopes")]
        public bool PreventSnappingOnLedges = false;
        /// <summary>
        /// The maximun downward slope angle change that the character can be subjected to and still be snapping to the ground
        /// </summary>    
        [Tooltip("The maximun downward slope angle change that the character can be subjected to and still be snapping to the ground")]
        [Range(1f, 180f)]
        public float MaxStableDenivelationAngle = 180f;

		/// <summary>
		/// Gravity applied to the character 
		/// </summary> 
		[Tooltip("Gravity applied to the character")]
		public Vector2 Gravity = new Vector2 (0, -30f);

		/// <summary>
		/// Drag applied to the character 
		/// </summary> 
		[Tooltip("Drag applied to the character")]
		public float Drag = 0.1f;

        /// <summary>
        /// How the character interacts with non-kinematic rigidbodies. \"Kinematic\" mode means the character pushes the rigidbodies with infinite force (as a kinematic body would). \"SimulatedDynamic\" pushes the rigidbodies with a simulated mass value.
        /// </summary>
		private RigidbodyInteractionType RigidbodyInteractionType = RigidbodyInteractionType.None;
        
		/// <summary>
        /// Determines if the character preserves moving platform velocities when de-grounding from them
        /// </summary>
        private bool PreserveAttachedRigidbodyMomentum = true;
        
        /// <summary>
        /// Handles properly detecting grounding status on steps, but has a performance cost.
        /// </summary>
        private StepHandlingMethod StepHandling = StepHandlingMethod.None;
        
		/// <summary>
        /// Handles properly detecting ledge information and grounding status, but has a performance cost.
        /// </summary>
        private bool LedgeHandling = false;
        
		/// <summary>
        /// Handles properly being pushed by and standing on PhysicsMovers or dynamic rigidbodies. Also handles pushing dynamic rigidbodies
        /// </summary>
        private bool InteractiveRigidbodyHandling = true;
       
		/// <summary>
        /// Makes sure the character cannot perform a move at all if it would be overlapping with any collidable objects at its destination. Useful for preventing \"tunneling\"
        /// </summary>
        private bool SafeMovement = false;

        /// <summary>
        /// Contains the current grounding information
        /// </summary>
        [System.NonSerialized]
        public CharacterGroundingReport GroundingStatus = new CharacterGroundingReport();

		/// <summary>
		/// Contains the current hit information
		/// </summary>
		[System.NonSerialized]
		public CharacterHitReport HitStatus = new CharacterHitReport();

        /// <summary>
        /// Contains the previous grounding information
        /// </summary>
        [System.NonSerialized]
        public CharacterTransientGroundingReport LastGroundingStatus = new CharacterTransientGroundingReport();
        /// <summary>
        /// Specifies the LayerMask that the character's movement algorithm can detect collisions with. By default, this uses the rigidbody's layer's collision matrix
        /// </summary>
        [System.NonSerialized]
        public LayerMask CollidableLayers = -1;

        /// <summary>
        /// The Transform of the character motor
        /// </summary>
        public Transform Transform { get; private set; }
        /// <summary>
        /// The character's up direction (always up-to-date during the character update phase)
        /// </summary>
        public Vector2 CharacterUp { get; private set; }
        /// <summary>
        /// The character's forward direction (always up-to-date during the character update phase)
        /// </summary>
        //public Vector3 CharacterForward { get; private set; }
        /// <summary>
        /// The character's right direction (always up-to-date during the character update phase)
        /// </summary>
        public Vector2 CharacterRight { get; private set; }
        /// <summary>
        /// The character's position before the movement calculations began
        /// </summary>
        public Vector2 InitialSimulationPosition { get; private set; }
        /// <summary>
        /// The character's rotation before the movement calculations began
        /// </summary>
        public Quaternion InitialSimulationRotation { get; private set; }
        /// <summary>
        /// Represents the Rigidbody to stay attached to
        /// </summary>
        public Rigidbody2D AttachedRigidbody { get; private set; }
        /// <summary>
        /// Vector2 from the character transform position to the capsule center
        /// </summary>
        public Vector2 CharacterTransformToCapsuleCenter { get; private set; }
        /// <summary>
        /// Vector3 from the character transform position to the capsule bottom
        /// </summary>
        public Vector3 CharacterTransformToCapsuleBottom { get; private set; }
        /// <summary>
        /// Vector3 from the character transform position to the capsule top
        /// </summary>
        public Vector3 CharacterTransformToCapsuleTop { get; private set; }
        /// <summary>
        /// Vector3 from the character transform position to the capsule bottom hemi center
        /// </summary>
        public Vector3 CharacterTransformToCapsuleBottomHemi { get; private set; }
        /// <summary>
        /// Vector3 from the character transform position to the capsule top hemi center
        /// </summary>
        public Vector3 CharacterTransformToCapsuleTopHemi { get; private set; }

        /// <summary>
        /// Is the motor trying to force unground?
        /// </summary>
        public bool MustUnground { get; set; }
        /// <summary>
        /// Did the motor's last swept collision detection find a ground?
        /// </summary>
        public bool LastMovementIterationFoundAnyGround { get; set; }
        /// <summary>
        /// Index of this motor in KinematicCharacterSystem arrays
        /// </summary>
        public int IndexInCharacterSystem { get; set; }
        /// <summary>
        /// Remembers initial position before all simulation are done
        /// </summary>
        public Vector3 InitialTickPosition { get; set; }
        /// <summary>
        /// Remembers initial rotation before all simulation are done
        /// </summary>
        public Quaternion InitialTickRotation { get; set; }
        /// <summary>
        /// Specifies a Rigidbody to stay attached to
        /// </summary>
        public Rigidbody2D AttachedRigidbodyOverride { get; set; }

        private RaycastHit2D[] _internalCharacterHits = new RaycastHit2D[MaxHitsBudget];
        private Collider2D[] _internalProbedColliders = new Collider2D[MaxCollisionBudget];
        private Rigidbody2D[] _rigidbodiesPushedThisMove = new Rigidbody2D[MaxCollisionBudget];
        private RigidbodyProjectionHit[] _internalRigidbodyProjectionHits = new RigidbodyProjectionHit[MaxMovementSweepIterations];
        private Rigidbody2D _lastAttachedRigidbody;
        private bool _solveMovementCollisions = true;
        private bool _solveGrounding = true;
        private bool _movePositionDirty = false;
        private Vector2 _movePositionTarget = Vector3.zero;
        private bool _moveRotationDirty = false;
        private Quaternion _moveRotationTarget = Quaternion.identity;
        private bool _lastSolvedOverlapNormalDirty = false;
        private Vector2 _lastSolvedOverlapNormal = Vector3.forward;
        private int _rigidbodiesPushedCount = 0;
        private int _rigidbodyProjectionHitCount = 0;
        private float _internalResultingMovementMagnitude = 0f;
        private Vector2 _internalResultingMovementDirection = Vector2.zero;
        private bool _isMovingFromAttachedRigidbody = false;
        private Vector2 _cachedWorldUp = Vector2.up;
        private Vector2 _cachedWorldRight = Vector2.right;
        private Vector2 _cachedZeroVector = Vector2.zero;
		private CapsuleDirection2D _capsuleDirection = CapsuleDirection2D.Vertical;
		private Vector2 _lastPosition;
		private bool _isSnappingToGround = false;
		private bool _startSnappingCourotine;


        private Vector2 _internalTransientPosition;
        /// <summary>
        /// The character's goal position in its movement calculations (always up-to-date during the character update phase)
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
        /// The character's goal rotation in its movement calculations (always up-to-date during the character update phase)
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
                CharacterUp = _internalTransientRotation * _cachedWorldUp;
//                CharacterForward = _internalTransientRotation * _cachedWorldForward;
                CharacterRight = _internalTransientRotation * _cachedWorldRight;
            }
        }

        /// <summary>
        /// The character's interpolated position
        /// </summary>
        public Vector2 InterpolatedPosition
        {
            get
            {
                return Transform.position;
            }
        }

        /// <summary>
        /// The character's interpolated rotation
        /// </summary>
        public Quaternion InterpolatedRotation
        {
            get
            {
                return Transform.rotation;
            }
        }

        /// <summary>
        /// The character's total velocity, including velocity from standing on rigidbodies or PhysicsMover
        /// </summary>
        public Vector2 Velocity
        {
            get
            {
                return _baseVelocity + _attachedRigidbodyVelocity;
            }
        }

        private Vector2 _baseVelocity;
        /// <summary>
        /// The character's velocity resulting from direct movement
        /// </summary>
        public Vector2 BaseVelocity
        {
            get
            {
                return _baseVelocity;
            }
            set
            {
                _baseVelocity = value;
            }
        }

        private Vector2 _attachedRigidbodyVelocity;
        /// <summary>
        /// The character's velocity resulting from standing on rigidbodies or PhysicsMover
        /// </summary>
        public Vector2 AttachedRigidbodyVelocity
        {
            get
            {
                return _attachedRigidbodyVelocity;
            }
            set
            {
                _attachedRigidbodyVelocity = value;
            }
        }

        /// <summary>
        /// The number of overlaps detected so far during character update (is reset at the beginning of the update)
        /// </summary>
        public int OverlapsCount { get; private set; }
        private OverlapResult[] _overlaps = new OverlapResult[MaxRigidbodyOverlapsCount];
        /// <summary>
        /// The overlaps detected so far during character update
        /// </summary>
        public OverlapResult[] Overlaps
        {
            get
            {
                return _overlaps;
            }
        }

        // Warning: Don't touch these constants unless you know exactly what you're doing!
        public const int MaxHitsBudget = 16;
        public const int MaxCollisionBudget = 16;
        public const int MaxGroundingSweepIterations = 2;
        public const int MaxMovementSweepIterations = 6;
        public const int MaxSteppingSweepIterations = 3;
        public const int MaxRigidbodyOverlapsCount = 16;
        public const int MaxDiscreteCollisionIterations = 3;
        public const float CollisionOffset = 0.001f;
        public const float GroundProbeReboundDistance = 0.02f;
        public const float MinimumGroundProbingDistance = 0.005f;
		public const float OneSideGroundProbingDistance = 0.2f;
		//public const float MinimumGroundProbingDistance = 0.1f;
        public const float GroundProbingBackstepDistance = 0.1f;
        public const float SweepProbingBackstepDistance = 0.002f;
        public const float SecondaryProbesVertical = 0.02f;
        public const float SecondaryProbesHorizontal = 0.001f;
        public const float MinVelocityMagnitude = 0.01f;
        public const float SteppingForwardDistance = 0.03f;
        public const float MinDistanceForLedge = 0.05f;
        public const float CorrelationForVerticalObstruction = 0.01f;
        public const float ExtraSteppingForwardDistance = 0.01f;
        public const float ExtraStepHeightPadding = 0.01f;
#pragma warning restore 0414 

        private void OnEnable()
        {
			CharacterControllerManager.EnsureCreation();
			CharacterControllerManager.RegisterCharacterMotor(this);
			//CharacterControllerManager.InterpolationMethod = CharacterManagerInterpolationMethod.Unity;
        }

        private void OnDisable()
        {
			CharacterControllerManager.UnregisterCharacterMotor(this);
        }

        private void Reset()
        {
            ValidateData();
        }

        private void OnValidate()
        {
            ValidateData();
        }

        [ContextMenu("Remove Component")]
        private void HandleRemoveComponent()
        {
            Rigidbody2D tmpRigidbody = gameObject.GetComponent<Rigidbody2D>();
            CapsuleCollider tmpCapsule = gameObject.GetComponent<CapsuleCollider>();
            DestroyImmediate(this);
            DestroyImmediate(tmpRigidbody);
            DestroyImmediate(tmpCapsule);
        }

        /// <summary>
        /// Handle validating all required values
        /// </summary>
        public void ValidateData()
        {
            Rigidbody = GetComponent<Rigidbody2D>();
            Rigidbody.centerOfMass = Vector3.zero;
            Rigidbody.drag = 0f;
            Rigidbody.angularDrag = 0f;
            Rigidbody.collisionDetectionMode = CollisionDetectionMode2D.Discrete;
            Rigidbody.isKinematic = true;
            Rigidbody.constraints = RigidbodyConstraints2D.None;
			Rigidbody.interpolation = CharacterControllerManager.InterpolationMethod == CharacterManagerInterpolationMethod.Unity ? RigidbodyInterpolation2D.Interpolate : RigidbodyInterpolation2D.None;

            Capsule = GetComponent<CapsuleCollider2D>();
            Capsule.isTrigger = false;

			if (CapsuleHeight >= CapsuleWidth) {
				_capsuleDirection = CapsuleDirection2D.Vertical;
			} else {
				_capsuleDirection = CapsuleDirection2D.Horizontal;
			}

			Capsule.direction = _capsuleDirection;
            Capsule.sharedMaterial = CapsulePhysicsMaterial;
			SetCapsuleDimensions(CapsuleWidth, CapsuleHeight, CapsuleYOffset);

            MaxStepHeight = Mathf.Clamp(MaxStepHeight, 0f, Mathf.Infinity);
			MinRequiredStepDepth = Mathf.Clamp(MinRequiredStepDepth, 0f, CapsuleWidth);

			MaxStableDistanceFromLedge = Mathf.Clamp(MaxStableDistanceFromLedge, 0f, CapsuleWidth);

            transform.localScale = Vector3.one;

#if UNITY_EDITOR
            Capsule.hideFlags = HideFlags.NotEditable;
            Rigidbody.hideFlags = HideFlags.NotEditable;
            if (!Mathf.Approximately(transform.lossyScale.x, 1f) || !Mathf.Approximately(transform.lossyScale.y, 1f) || !Mathf.Approximately(transform.lossyScale.z, 1f))
            {
                Debug.LogError("Character's lossy scale is not (1,1,1). This is not allowed. Make sure the character's transform and all of its parents have a (1,1,1) scale.", this.gameObject);
            }
#endif
        }

        /// <summary>
        /// Sets whether or not the capsule collider will detect collisions
        /// </summary>
        public void SetCapsuleCollisionsActivation(bool kinematicCapsuleActive)
        {
            //Rigidbody.detectCollisions = kinematicCapsuleActive;
        }

        /// <summary>
        /// Sets whether or not the motor will solve collisions when moving (or moved onto)
        /// </summary>
        public void SetMovementCollisionsSolvingActivation(bool movementCollisionsSolvingActive)
        {
            _solveMovementCollisions = movementCollisionsSolvingActive;
        }

        /// <summary>
        /// Sets whether or not grounding will be evaluated for all hits
        /// </summary>
        public void SetGroundSolvingActivation(bool stabilitySolvingActive)
        {
            _solveGrounding = stabilitySolvingActive;
        }

        /// <summary>
        /// Sets the character's position directly
        /// </summary>
        public void SetPosition(Vector3 position, bool bypassInterpolation = true)
        {
            Rigidbody.interpolation = RigidbodyInterpolation2D.None;
            Transform.position = position;
            Rigidbody.position = position;
            InitialSimulationPosition = position;
            TransientPosition = position;

            if (bypassInterpolation)
            {
                InitialTickPosition = position;
            }

			Rigidbody.interpolation = CharacterControllerManager.InterpolationMethod == CharacterManagerInterpolationMethod.Unity ? RigidbodyInterpolation2D.Interpolate : RigidbodyInterpolation2D.None;
        }

        /// <summary>
        /// Sets the character's rotation directly
        /// </summary>
        public void SetRotation(Quaternion rotation, bool bypassInterpolation = true)
        {
            Rigidbody.interpolation = RigidbodyInterpolation2D.None;
            Transform.rotation = rotation;
            Rigidbody.rotation = rotation.z;
            InitialSimulationRotation = rotation;
            TransientRotation = rotation;

            if (bypassInterpolation)
            {
                InitialTickRotation = rotation;
            }

			Rigidbody.interpolation = CharacterControllerManager.InterpolationMethod == CharacterManagerInterpolationMethod.Unity ? RigidbodyInterpolation2D.Interpolate : RigidbodyInterpolation2D.None;
        }

        /// <summary>
        /// Sets the character's position and rotation directly
        /// </summary>
        public void SetPositionAndRotation(Vector3 position, Quaternion rotation, bool bypassInterpolation = true)
        {
            Rigidbody.interpolation = RigidbodyInterpolation2D.None;
            Transform.SetPositionAndRotation(position, rotation);
            Rigidbody.position = position;
            Rigidbody.rotation = rotation.z;
            InitialSimulationPosition = position;
            InitialSimulationRotation = rotation;
            TransientPosition = position;
            TransientRotation = rotation;

            if (bypassInterpolation)
            {
                InitialTickPosition = position;
                InitialTickRotation = rotation;
            }

			Rigidbody.interpolation = CharacterControllerManager.InterpolationMethod == CharacterManagerInterpolationMethod.Unity ? RigidbodyInterpolation2D.Interpolate : RigidbodyInterpolation2D.None;
        }

        /// <summary>
        /// Moves the character position, taking all movement collision solving int account. The actual move is done the next time the motor updates are called
        /// </summary>
        public void MoveCharacter(Vector3 toPosition)
        {
            _movePositionDirty = true;
            _movePositionTarget = toPosition;
        }

        /// <summary>
        /// Moves the character rotation. The actual move is done the next time the motor updates are called
        /// </summary>
        public void RotateCharacter(Quaternion toRotation)
        {
            _moveRotationDirty = true;
            _moveRotationTarget = toRotation;
        }

        /// <summary>
        /// Returns all the state information of the motor that is pertinent for simulation
        /// </summary>
		public CharacterState GetState()
        {
			CharacterState state = new CharacterState();

            state.Position = TransientPosition;
            state.Rotation = TransientRotation;

            state.BaseVelocity = _baseVelocity;
            state.AttachedRigidbodyVelocity = _attachedRigidbodyVelocity;

            state.MustUnground = MustUnground;
            state.LastMovementIterationFoundAnyGround = LastMovementIterationFoundAnyGround;
            state.GroundingStatus.CopyFrom(GroundingStatus);
            state.AttachedRigidbody = AttachedRigidbody;

            return state;
        }

        /// <summary>
        /// Applies a motor state instantly
        /// </summary>
		public void ApplyState(CharacterState state, bool bypassInterpolation = true)
        {
            SetPositionAndRotation(state.Position, state.Rotation, bypassInterpolation);

            BaseVelocity = state.BaseVelocity;
            AttachedRigidbodyVelocity = state.AttachedRigidbodyVelocity;

            MustUnground = state.MustUnground;
            LastMovementIterationFoundAnyGround = state.LastMovementIterationFoundAnyGround;
            GroundingStatus.CopyFrom(state.GroundingStatus);
            AttachedRigidbody = state.AttachedRigidbody;
        }

        /// <summary>
        /// Resizes capsule. ALso caches importand capsule size data
        /// </summary>
        public void SetCapsuleDimensions(float width, float height, float yOffset)
        {
			CapsuleWidth = width;
            CapsuleHeight = height;
            CapsuleYOffset = yOffset;

			Capsule.offset = new Vector2 (0f, CapsuleYOffset);
			Capsule.size = new Vector2 (CapsuleWidth, CapsuleHeight);

            CharacterTransformToCapsuleCenter = Capsule.offset;
			CharacterTransformToCapsuleBottom = Capsule.offset + (-_cachedWorldUp * (Capsule.size.y * 0.5f));
			CharacterTransformToCapsuleTop = Capsule.offset + (_cachedWorldUp * (Capsule.size.y * 0.5f));
			CharacterTransformToCapsuleBottomHemi = Capsule.offset + (-_cachedWorldUp * (Capsule.size.y * 0.5f)) + (_cachedWorldUp * Capsule.size.x);
			CharacterTransformToCapsuleTopHemi = Capsule.offset + (_cachedWorldUp * (Capsule.size.y * 0.5f)) + (-_cachedWorldUp * Capsule.size.x);

		}

        private void Awake()
        {
            Transform = this.transform;
            ValidateData();

            TransientPosition = Transform.position;
            TransientRotation = Transform.rotation;

            // Build CollidableLayers mask
            CollidableLayers = 0;
            for (int i = 0; i < 32; i++)
            {
                if (!Physics.GetIgnoreLayerCollision(this.gameObject.layer, i))
                {
                    CollidableLayers |= (1 << i);
                }
            }

            if(CharacterController)
            {
                CharacterController.SetupCharacterMotor(this);
            }

			SetCapsuleDimensions(CapsuleWidth, CapsuleHeight, CapsuleYOffset);
        }

        /// <summary>
        /// Update phase 1 is meant to be called after physics movers have calculated their velocities, but
        /// before they have simulated their goal positions/rotations. It is responsible for:
        /// - Initializing all values for update
        /// - Handling MovePosition calls
        /// - Solving initial collision overlaps
        /// - Ground probing
        /// - Handle detecting potential interactable rigidbodies
        /// </summary>
        public void UpdatePhase1(float deltaTime)
        {
			
            // NaN propagation safety stop
            if (float.IsNaN(_baseVelocity.x) || float.IsNaN(_baseVelocity.y))
            {
                _baseVelocity = Vector3.zero;
            }
	

            if (float.IsNaN(_attachedRigidbodyVelocity.x) || float.IsNaN(_attachedRigidbodyVelocity.y))
            {
                _attachedRigidbodyVelocity = Vector3.zero;
            }

#if UNITY_EDITOR
            if (!Mathf.Approximately(Transform.lossyScale.x, 1f) || !Mathf.Approximately(Transform.lossyScale.y, 1f) || !Mathf.Approximately(Transform.lossyScale.z, 1f))
            {
                //Debug.LogError("Character's lossy scale is not (1,1,1). This is not allowed. Make sure the character's transform and all of its parents have a (1,1,1) scale.", this.gameObject);
            }
#endif
            
            // Before update
            this.CharacterController.BeforeCharacterUpdate(deltaTime);

            TransientPosition = Transform.position;
            TransientRotation = Transform.rotation;
            InitialSimulationPosition = TransientPosition;
            InitialSimulationRotation = TransientRotation;
            _rigidbodyProjectionHitCount = 0;
            OverlapsCount = 0;

            _lastSolvedOverlapNormalDirty = false;

            #region Handle Move Position
            if (_movePositionDirty)
            {
                if (_solveMovementCollisions)
                {
                    if (InternalCharacterMove((_movePositionTarget - TransientPosition), deltaTime, out _internalResultingMovementMagnitude, out _internalResultingMovementDirection))
                    {
                        if (InteractiveRigidbodyHandling)
                        {
                            Vector2 tmpVelocity = Vector2.zero;
                            ProcessVelocityForRigidbodyHits(ref tmpVelocity, deltaTime);
                        }
                    }
                }
                else
                {
                    TransientPosition = _movePositionTarget;
                }

                _movePositionDirty = false;
            }
            #endregion

            LastGroundingStatus.CopyFrom(GroundingStatus);
            GroundingStatus = new CharacterGroundingReport();
            GroundingStatus.GroundNormal = CharacterUp;




            if (_solveMovementCollisions)
            {
                #region Resolve initial overlaps
                Vector3 resolutionDirection = _cachedWorldUp;
                float resolutionDistance = 0f;
                int iterationsMade = 0;
                bool overlapSolved = false;
                while(iterationsMade < MaxDiscreteCollisionIterations && !overlapSolved)
                {
                    int nbOverlaps = CharacterCollisionsOverlap(TransientPosition, TransientRotation, _internalProbedColliders);

                    if (nbOverlaps > 0)
                    {
						
                        // Solve overlaps that aren't against dynamic rigidbodies or physics movers
                        for (int i = 0; i < nbOverlaps; i++)
                        {
                            Rigidbody2D probedRigidbody = _internalProbedColliders[i].attachedRigidbody;
                            bool isPhysicsMoverOrDynamicRigidbody = probedRigidbody && (!probedRigidbody.isKinematic || probedRigidbody.GetComponent<PhysicsMover>());
                            if (!isPhysicsMoverOrDynamicRigidbody)
                            {
                                // Process overlap
                                Transform overlappedTransform = _internalProbedColliders[i].transform;

								ColliderDistance2D dist = Capsule.Distance(_internalProbedColliders[i]);
								resolutionDistance = dist.distance;
								resolutionDirection = (dist.pointA - dist.pointB).normalized;

								if(AllowOneSide(TransientPosition, TransientRotation, _internalProbedColliders[i], dist.pointB, resolutionDirection.normalized))
								{
                                    // Resolve along obstruction direction

                                   // Vector2 originalResolutionDirection = resolutionDirection;
                                    HitStabilityReport mockReport = new HitStabilityReport();
                                    mockReport.IsStable = IsStableOnNormal(resolutionDirection);
                                    resolutionDirection = GetObstructionNormal(resolutionDirection, mockReport);

                                    // Solve overlap
                                    Vector2 resolutionMovement = resolutionDirection * (resolutionDistance + CollisionOffset);
                                    TransientPosition += resolutionMovement;

                                    // Remember overlaps
                                    if (OverlapsCount < _overlaps.Length)
                                    {
                                       _overlaps[OverlapsCount] = new OverlapResult(resolutionDirection, _internalProbedColliders[i]);
                                        OverlapsCount++;
                                    }

                                    break;
								}
                               
                            }
                        }
                    }
                    else
                    {
                        overlapSolved = true;
                    }

                    iterationsMade++;
                }
                #endregion
            }

            #region Ground Probing and Snapping
            // Handle ungrounding
            if (_solveGrounding)
            {
				if (MustUnground)
                {
                    TransientPosition += CharacterUp * (MinimumGroundProbingDistance * 1.5f);
                }
                else
                {
                    // Choose the appropriate ground probing distance
					float selectedGroundProbingDistance = MinimumGroundProbingDistance; 
                    if (!LastGroundingStatus.SnappingPrevented && (LastGroundingStatus.IsStableOnGround || LastMovementIterationFoundAnyGround))
                    {
                        if (StepHandling != StepHandlingMethod.None)
                        {
							selectedGroundProbingDistance = Mathf.Max(CapsuleWidth, MaxStepHeight);
                        }
                        else
                        {
							selectedGroundProbingDistance = CapsuleWidth;
                        }

                        selectedGroundProbingDistance += GroundDetectionExtraDistance;
                    }

			
                    ProbeGround(ref _internalTransientPosition, TransientRotation, selectedGroundProbingDistance, ref GroundingStatus);

                }
            }

            LastMovementIterationFoundAnyGround = false;
            MustUnground = false;
            #endregion

            if (_solveGrounding)
            {
                CharacterController.PostGroundingUpdate(deltaTime);
            }

            if(InteractiveRigidbodyHandling)
            {
                #region Interactive Rigidbody Handling 
                _lastAttachedRigidbody = AttachedRigidbody;
                if (AttachedRigidbodyOverride)
                {
                    AttachedRigidbody = AttachedRigidbodyOverride;
                }
                else
                {
                    // Detect interactive rigidbodies from grounding
                    if (GroundingStatus.IsStableOnGround && GroundingStatus.GroundCollider.attachedRigidbody)
                    {
                        Rigidbody2D interactiveRigidbody = GetInteractiveRigidbody(GroundingStatus.GroundCollider);
                        if (interactiveRigidbody)
                        {
                            AttachedRigidbody = interactiveRigidbody;
                        }
                    }
                    else
                    {
                        AttachedRigidbody = null;
                    }
                }

                Vector2 tmpVelocityFromCurrentAttachedRigidbody = Vector2.zero;
                if(AttachedRigidbody)
                {
                    tmpVelocityFromCurrentAttachedRigidbody = GetVelocityFromRigidbodyMovement(AttachedRigidbody, TransientPosition, deltaTime);
                }

                // Conserve momentum when de-stabilized from an attached rigidbody
                if (PreserveAttachedRigidbodyMomentum && _lastAttachedRigidbody != null && AttachedRigidbody != _lastAttachedRigidbody)
                {
                    _baseVelocity += _attachedRigidbodyVelocity;
                    _baseVelocity -= tmpVelocityFromCurrentAttachedRigidbody;
                }

                // Process additionnal Velocity from attached rigidbody
                _attachedRigidbodyVelocity = _cachedZeroVector;
                if (AttachedRigidbody)
                {
                    _attachedRigidbodyVelocity = tmpVelocityFromCurrentAttachedRigidbody;

                    // Rotation from attached rigidbody
					Vector3 newForward = Vector3.ProjectOnPlane(Quaternion.Euler(Mathf.Rad2Deg * (Vector3.forward * AttachedRigidbody.angularVelocity)  * deltaTime) * Vector3.forward, CharacterUp).normalized;
                    TransientRotation = Quaternion.LookRotation(newForward, CharacterUp);
                }

                // Cancel out horizontal velocity upon landing on an attached rigidbody
                if (GroundingStatus.GroundCollider &&
                    GroundingStatus.GroundCollider.attachedRigidbody && 
                    GroundingStatus.GroundCollider.attachedRigidbody == AttachedRigidbody && 
                    AttachedRigidbody != null && 
                    _lastAttachedRigidbody == null)
                {
					Vector3 prj = Vector3.ProjectOnPlane(_attachedRigidbodyVelocity, CharacterUp);
                    //_baseVelocity -=  
					_baseVelocity -= new Vector2(prj.x, prj.y);
                }

                // Movement from Attached Rigidbody
                if (_attachedRigidbodyVelocity.sqrMagnitude > 0f)
                {
                    _isMovingFromAttachedRigidbody = true;

                    if (_solveMovementCollisions)
                    {
                        // Perform the move from rgdbdy velocity
                        if (InternalCharacterMove(_attachedRigidbodyVelocity * deltaTime, deltaTime, out _internalResultingMovementMagnitude, out _internalResultingMovementDirection))
                        {
                            _attachedRigidbodyVelocity = (_internalResultingMovementDirection * _internalResultingMovementMagnitude) / deltaTime;
                        }
                        else
                        {
                            _attachedRigidbodyVelocity = Vector3.zero;
                        }
                    }
                    else
                    {
                        TransientPosition += _attachedRigidbodyVelocity * deltaTime;
                    }
                    
                    _isMovingFromAttachedRigidbody = false;
                }
                #endregion
            }
        }

        /// <summary>
        /// Update phase 2 is meant to be called after physics movers have simulated their goal positions/rotations. 
        /// At the end of this, the TransientPosition/Rotation values will be up-to-date with where the motor should be at the end of its move. 
        /// It is responsible for:
        /// - Solving Rotation
        /// - Handle MoveRotation calls
        /// - Solving potential attached rigidbody overlaps
        /// - Solving Velocity
        /// - Applying planar constraint
        /// </summary>
        public void UpdatePhase2(float deltaTime)
        {
			_lastPosition = TransientPosition;

            // Handle rotation
            this.CharacterController.UpdateRotation(ref _internalTransientRotation, deltaTime);
            TransientRotation = _internalTransientRotation;

            // Handle move rotation
            if (_moveRotationDirty)
            {
                TransientRotation = _moveRotationTarget;
                _moveRotationDirty = false;
            }
            
            if (_solveMovementCollisions && InteractiveRigidbodyHandling)
            {
                if (InteractiveRigidbodyHandling)
                {
                    #region Solve potential attached rigidbody overlap
                    if (AttachedRigidbody)
                    {
                        float upwardsOffset = Capsule.size.x;

                        RaycastHit2D closestHit;
                        if (CharacterGroundSweep(
                            TransientPosition + (CharacterUp * upwardsOffset),
                            TransientRotation.z,
                            -CharacterUp,
                            upwardsOffset,
                            out closestHit))
                        {
                            if (closestHit.collider.attachedRigidbody == AttachedRigidbody && IsStableOnNormal(closestHit.normal))
                            {
                                float distanceMovedUp = (upwardsOffset - closestHit.distance);
                                TransientPosition = TransientPosition + (CharacterUp * distanceMovedUp) + (CharacterUp * CollisionOffset);
                            }
                        }
                    }
                    #endregion
                }

                if (SafeMovement || InteractiveRigidbodyHandling)
                {
                    #region Resolve overlaps that could've been caused by rotation or physics movers simulation pushing the character
                    Vector3 resolutionDirection = _cachedWorldUp;
                    float resolutionDistance = 0f;
                    int iterationsMade = 0;
                    bool overlapSolved = false;
                    while (iterationsMade < MaxDiscreteCollisionIterations && !overlapSolved)
                    {
                        int nbOverlaps = CharacterCollisionsOverlap(TransientPosition, TransientRotation, _internalProbedColliders,0, false);
                        if (nbOverlaps > 0)
                        {
                            for (int i = 0; i < nbOverlaps; i++)
                            {
                                // Process overlap
                                //Transform overlappedTransform = _internalProbedColliders[i].GetComponent<Transform>();

								ColliderDistance2D dist = Capsule.Distance(_internalProbedColliders[i]);


								resolutionDistance = dist.distance;
									resolutionDirection = (dist.pointA - dist.pointB).normalized;
							

								bool allowOneSide = AllowOneSide(TransientPosition, TransientRotation,_internalProbedColliders[i], dist.pointB, resolutionDirection.normalized, 0);
								bool isPhysicsMoverOrDynamicRigidbody = false; 

								float effectiveOneSideGroundProbingDistance = OneSideGroundProbingDistance;
								HitStabilityReport mockReport = new HitStabilityReport();
								mockReport.IsStable = IsStableOnNormal(resolutionDirection);
								resolutionDirection = GetObstructionNormal(resolutionDirection, mockReport);

								// Solve overlap
								Vector2 resolutionMovement = resolutionDirection * (resolutionDistance + CollisionOffset);

								if(_baseVelocity.normalized.y <= 0 && IsOneSideGround(_internalProbedColliders[i]) && _internalProbedColliders[i].attachedRigidbody != null)
								{
									effectiveOneSideGroundProbingDistance = Mathf.Max(CapsuleWidth, MaxStepHeight);
								}

		
								if((allowOneSide) || (!allowOneSide && (-resolutionDistance <= effectiveOneSideGroundProbingDistance) && resolutionDirection.normalized.y < 0))
								{
                                    TransientPosition += resolutionMovement;
                                    // If physicsMover, register as rigidbody hit for velocity
                                    if (InteractiveRigidbodyHandling)
                                    {
                                        Rigidbody2D probedRigidbody = _internalProbedColliders[i].attachedRigidbody;
                                        if (probedRigidbody)
                                        {
                                            PhysicsMover physicsMover = probedRigidbody.GetComponent<PhysicsMover>();
                                            if (physicsMover)
                                            {
                                                 isPhysicsMoverOrDynamicRigidbody = probedRigidbody && (!probedRigidbody.isKinematic || physicsMover);
                                                if (isPhysicsMoverOrDynamicRigidbody)
                                                {
                                                    HitStabilityReport tmpReport = new HitStabilityReport();
                                                    tmpReport.IsStable = IsStableOnNormal(resolutionDirection);
	
													if (tmpReport.IsStable)
                                                    {
                                                        LastMovementIterationFoundAnyGround = tmpReport.IsStable;
                                                    }
                                                    if (physicsMover.Rigidbody && physicsMover.Rigidbody != AttachedRigidbody)
                                                    {
														Vector2 rot = TransientRotation * new Vector3(CharacterTransformToCapsuleCenter.x, CharacterTransformToCapsuleCenter.y, 0);
                                                        //Vector2 characterCenter = TransientPosition + (TransientRotation.z * CharacterTransformToCapsuleCenter);
														Vector2 estimatedCollisionPoint = TransientPosition + rot;


                                                        StoreRigidbodyHit(
                                                            physicsMover.Rigidbody,
                                                            Velocity,
                                                            estimatedCollisionPoint,
                                                            resolutionDirection,
                                                            tmpReport);
                                                    }
                                                }
                                            }
                                        }
                                    }
								
								
									if(allowOneSide)
									{
	                                    // Remember overlaps
										if (OverlapsCount < _overlaps.Length && !isPhysicsMoverOrDynamicRigidbody)
	                                    {
	                                        _overlaps[OverlapsCount] = new OverlapResult( resolutionDirection, _internalProbedColliders[i]);
	                                        OverlapsCount++;
	                                    }
									}

								}
								
                                    break;
                            
							}
                        }
                        else
                        {
                            overlapSolved = true;
                        }

                        iterationsMade++;
                    }
                    #endregion
                }
            }
				
			// Handle velocity
            this.CharacterController.UpdateVelocity(ref _baseVelocity, deltaTime);
			HitStatus.FoundAnyHit = false;
			HitStatus.HitCollider = null;
			HitStatus.HitNormal = Vector2.zero;
			HitStatus.HitPoint = Vector2.zero;

			//Apply gravity
			if (!GroundingStatus.IsStableOnGround) {
				// Gravity
				_baseVelocity += Gravity * deltaTime;

				// Drag
				_baseVelocity *= (1f / (1f + (Drag * deltaTime)));
			}
            
			if (_baseVelocity.magnitude < MinVelocityMagnitude)
            {
                _baseVelocity = Vector3.zero;
            }

            #region Calculate Character movement from base velocity   
            // Perform the move from base velocity
            if (_baseVelocity.sqrMagnitude > 0f)
            {
                if (_solveMovementCollisions)
                {
                    if (InternalCharacterMove(_baseVelocity * deltaTime, deltaTime, out _internalResultingMovementMagnitude, out _internalResultingMovementDirection))
                    {
                        _baseVelocity = (_internalResultingMovementDirection * _internalResultingMovementMagnitude) / deltaTime;
                    }
                    else
                    {
                        _baseVelocity = Vector3.zero;
                    }
                }
                else
                {
                    TransientPosition += _baseVelocity * deltaTime;
                }
            }

            // Process rigidbody hits/overlaps to affect velocity
            if (InteractiveRigidbodyHandling)
            {
                ProcessVelocityForRigidbodyHits(ref _baseVelocity, deltaTime);
            }
            #endregion


            // Discrete collision detection
            if(DetectDiscreteCollisions)
            {
                int nbOverlaps = CharacterCollisionsOverlap(TransientPosition, TransientRotation, _internalProbedColliders, CollisionOffset * 2f);
                for(int i = 0; i < nbOverlaps; i++)
                {
                    CharacterController.OnDiscreteCollisionDetected(_internalProbedColliders[i]);
                }
            }

            this.CharacterController.AfterCharacterUpdate(deltaTime);

        }

        /// <summary>
        /// Determines if motor can be considered stable on given slope normal
        /// </summary>
        private bool IsStableOnNormal(Vector3 normal)
        {
            return Vector3.Angle(CharacterUp, normal) <= MaxStableSlopeAngle;
        }

        /// <summary>
        /// Probes for valid ground and midifies the input transientPosition if ground snapping occurs
        /// </summary>
        public void ProbeGround(ref Vector2 probingPosition, Quaternion atRotation, float probingDistance, ref CharacterGroundingReport groundingReport)
        {
            if (probingDistance < MinimumGroundProbingDistance)
            {
                probingDistance = MinimumGroundProbingDistance;
            }

            int groundSweepsMade = 0;
			RaycastHit2D groundSweepHit = new RaycastHit2D();
            bool groundSweepingIsOver = false;
            Vector2 groundSweepPosition = probingPosition;
            Vector2 groundSweepDirection = (atRotation * -_cachedWorldUp);
            float groundProbeDistanceRemaining = probingDistance;
            while (groundProbeDistanceRemaining > 0 && (groundSweepsMade <= MaxGroundingSweepIterations) && !groundSweepingIsOver)
            {
                // Sweep for ground detection
                if (CharacterGroundSweep(
                        groundSweepPosition, // position
                        atRotation.z, // rotation
                        groundSweepDirection, // direction
                        groundProbeDistanceRemaining, // distance
                        out groundSweepHit)) // hit
                {
                    Vector2 targetPosition = groundSweepPosition + (groundSweepDirection * groundSweepHit.distance);
                    HitStabilityReport groundHitStabilityReport = new HitStabilityReport();
                    EvaluateHitStability(groundSweepHit.collider, groundSweepHit.normal, groundSweepHit.point, targetPosition, TransientRotation, ref groundHitStabilityReport);
					//groundHitStabilityReport.IsStable = true;
                    // Handle ledge stability
                    if (groundHitStabilityReport.LedgeDetected)
                    {
                        if (groundHitStabilityReport.IsOnEmptySideOfLedge && groundHitStabilityReport.DistanceFromLedge > MaxStableDistanceFromLedge)
                        {
                            groundHitStabilityReport.IsStable = false;
                        }
                    }

                    groundingReport.FoundAnyGround = true;
                    groundingReport.GroundNormal = groundSweepHit.normal;
                    groundingReport.InnerGroundNormal = groundHitStabilityReport.InnerNormal;
                    groundingReport.OuterGroundNormal = groundHitStabilityReport.OuterNormal;
                    groundingReport.GroundCollider = groundSweepHit.collider;
                    groundingReport.GroundPoint = groundSweepHit.point;
                    groundingReport.SnappingPrevented = false;

                    // Found stable ground
                    if (groundHitStabilityReport.IsStable)
                    {
                        // Find all scenarios where ground snapping should be canceled
                        if (LedgeHandling)
                        {
                            // "Launching" off of slopes of a certain denivelation angle
                            if (LastGroundingStatus.FoundAnyGround && groundHitStabilityReport.InnerNormal.sqrMagnitude != 0f && groundHitStabilityReport.OuterNormal.sqrMagnitude != 0f)
                            {
                                float denivelationAngle = Vector2.Angle(groundHitStabilityReport.InnerNormal, groundHitStabilityReport.OuterNormal);
                                if (denivelationAngle > MaxStableDenivelationAngle)
                                {
                                    groundingReport.SnappingPrevented = true;
                                }
                                else
                                {
                                    denivelationAngle = Vector2.Angle(LastGroundingStatus.InnerGroundNormal, groundHitStabilityReport.OuterNormal);
                                    if (denivelationAngle > MaxStableDenivelationAngle)
                                    {
                                        groundingReport.SnappingPrevented = true;
                                    }
                                }
                            }

                            // Ledge stability
                            if (PreventSnappingOnLedges && groundHitStabilityReport.LedgeDetected)
                            {
                                groundingReport.SnappingPrevented = true;
                            }
                        }

                        groundingReport.IsStableOnGround = true;

                        // Ground snapping
                        if (!groundingReport.SnappingPrevented)
                        {
							if (_isSnappingToGround) {
								targetPosition += (-groundSweepDirection * CollisionOffset);
								InternalMoveCharacterPosition (ref probingPosition, targetPosition, atRotation);
							}
                        }

                        this.CharacterController.OnGroundHit(groundSweepHit.collider, groundSweepHit.normal, groundSweepHit.point, ref groundHitStabilityReport);
                        groundSweepingIsOver = true;

						if (!_isSnappingToGround) {
							if (_baseVelocity.y > 0 && IsOneSideGround(GroundingStatus.GroundCollider)) {
								if (!_startSnappingCourotine) {
									StartCoroutine (SetSnappingCourotine ());
									_startSnappingCourotine = true;
								}
							} else {
								_isSnappingToGround = true;
							}
						}
                    }
                    else
                    {
						Vector2 rot = (atRotation * Vector3.up);
                        // Calculate movement from this iteration and advance position
						Vector2 sweepMovement = (groundSweepDirection * groundSweepHit.distance) + (rot * Mathf.Clamp(CollisionOffset, 0f, groundSweepHit.distance));
                        groundSweepPosition = groundSweepPosition + sweepMovement;

                        // Set remaining distance
                        groundProbeDistanceRemaining = Mathf.Min(GroundProbeReboundDistance, Mathf.Clamp(groundProbeDistanceRemaining - sweepMovement.magnitude, 0f, Mathf.Infinity));

                        // Reorient direction
                        groundSweepDirection = Vector3.ProjectOnPlane(groundSweepDirection, groundSweepHit.normal).normalized;

                    }
                }
                else
                {
                    groundSweepingIsOver = true;
                }

				groundSweepsMade++;
            }

        }

		IEnumerator SetSnappingCourotine()
		{
			yield return new WaitForSeconds (0.1f);
			_isSnappingToGround = true;
			_startSnappingCourotine = false;
		}

        /// <summary>
        /// Forces the character to unground itself on its next grounding update
        /// </summary>
        public void ForceUnground()
        {
            MustUnground = true;
			_isSnappingToGround = false;
        }

        /// <summary>
        /// Returns the direction adjusted to be tangent to a specified surface normal relatively to the character's up direction.
        /// Useful for reorienting a direction on a slope without any lateral deviation in trajectory
        /// </summary>
        public Vector3 GetDirectionTangentToSurface(Vector3 direction, Vector3 surfaceNormal)
        {
            Vector3 directionRight = Vector3.Cross(direction, CharacterUp);
            return Vector3.Cross(surfaceNormal, directionRight).normalized;
        }
			
        /// <summary>
        /// Moves the character's position by given movement while taking into account all physics simulation, step-handling and 
        /// velocity projection rules that affect the character motor
        /// </summary>
        /// <returns> Returns false if movement could not be solved until the end </returns>
        private bool InternalCharacterMove(Vector2 movement, float deltaTime, out float resultingMovementMagnitude, out Vector2 resultingMovementDirection)
        {
            _rigidbodiesPushedCount = 0;
            bool wasCompleted = true;
            Vector2 remainingMovementDirection = movement.normalized;
            float remainingMovementMagnitude = movement.magnitude;
            resultingMovementDirection = remainingMovementDirection;
            resultingMovementMagnitude = remainingMovementMagnitude;
            int sweepsMade = 0;
            RaycastHit2D closestSweepHit;
            bool hitSomethingThisSweepIteration = true;
            Vector2 tmpMovedPosition = TransientPosition;
            Vector2 targetPositionAfterSweep = TransientPosition;
            Vector2 originalMoveDirection = movement.normalized;
            Vector2 previousMovementHitNormal = _cachedZeroVector;
            MovementSweepState sweepState = MovementSweepState.Initial;


			if (!MustUnground) {
				// Project movement against current overlaps
				for (int i = 0; i < OverlapsCount; i++) {
					if (Vector2.Dot (remainingMovementDirection, _overlaps [i].Normal) < 0f) {
						InternalHandleMovementProjection (
							IsStableOnNormal (
								_overlaps [i].Normal) && !MustUnground,
							_overlaps [i].Normal,
							_overlaps [i].Normal,
							originalMoveDirection,
							ref sweepState,
							ref previousMovementHitNormal,
							ref resultingMovementMagnitude,
							ref remainingMovementDirection,
							ref remainingMovementMagnitude);
					}
				}
			}

			/*if (OverlapsCount == 0) {
				HitStatus.FoundAnyHit = false;
			} else {
				HitStatus.FoundAnyHit = true;
				Debug.Log ("d");
			}*/

            // Sweep the desired movement to detect collisions
            while (remainingMovementMagnitude > 0f &&
                (sweepsMade <= MaxMovementSweepIterations) &&
                hitSomethingThisSweepIteration)
            {
                if (CharacterCollisionsSweep(
                        tmpMovedPosition, // position
                        TransientRotation, // rotation
                        remainingMovementDirection, // direction
                        remainingMovementMagnitude + CollisionOffset, // distance
                        out closestSweepHit, // closest hit
                        _internalCharacterHits) // all hits
                    > 0)
                {

                    // Calculate movement from this iteration
                    targetPositionAfterSweep = tmpMovedPosition + (remainingMovementDirection * closestSweepHit.distance) + (closestSweepHit.normal * CollisionOffset);
                    Vector3 sweepMovement = targetPositionAfterSweep - tmpMovedPosition;


                    // Evaluate if hit is stable
                    HitStabilityReport moveHitStabilityReport = new HitStabilityReport();
                    EvaluateHitStability(closestSweepHit.collider, closestSweepHit.normal, closestSweepHit.point, targetPositionAfterSweep, TransientRotation, ref moveHitStabilityReport);
                    // Handle stepping up perfectly vertical walls
                    bool foundValidStepHit = false;
                    if (_solveGrounding && StepHandling != StepHandlingMethod.None && moveHitStabilityReport.ValidStepDetected)
                    {
                        float obstructionCorrelation = Mathf.Abs(Vector3.Dot(closestSweepHit.normal, CharacterUp));
                        if (obstructionCorrelation <= CorrelationForVerticalObstruction)
                        {
                            RaycastHit2D closestStepHit;
                            Vector2 stepForwardDirection = Vector3.ProjectOnPlane(-closestSweepHit.normal, CharacterUp).normalized;
                            Vector2 stepCastStartPoint = (targetPositionAfterSweep + (stepForwardDirection * SteppingForwardDistance)) +
                                (CharacterUp * MaxStepHeight);

                            // Cast downward from the top of the stepping height
                            int nbStepHits = CharacterCollisionsSweep(
                                                stepCastStartPoint, // position
                                                TransientRotation, // rotation
                                                -CharacterUp, // direction
                                                MaxStepHeight, // distance
                                                out closestStepHit, // closest hit
                                                _internalCharacterHits); // all hitswwasa  

                            // Check for hit corresponding to stepped collider
                            for (int i = 0; i < nbStepHits; i++)
                            {
                                if (_internalCharacterHits[i].collider == moveHitStabilityReport.SteppedCollider)
                                {

                                    Vector3 endStepPosition = stepCastStartPoint + (-CharacterUp * (_internalCharacterHits[i].distance - CollisionOffset));
                                    tmpMovedPosition = endStepPosition;
                                    foundValidStepHit = true;

                                    // Consume magnitude for step
                                    remainingMovementMagnitude = Mathf.Clamp(remainingMovementMagnitude - sweepMovement.magnitude, 0f, Mathf.Infinity);
                                    break;
                                }
                            }
                        }
                    }

                    // Handle movement solving
					if (!foundValidStepHit) {
						// Apply the actual movement
						tmpMovedPosition = targetPositionAfterSweep;
						remainingMovementMagnitude = Mathf.Clamp (remainingMovementMagnitude - sweepMovement.magnitude, 0f, Mathf.Infinity);
                        
						// Movement hit callback
						this.CharacterController.OnMovementHit (closestSweepHit.collider, closestSweepHit.normal, closestSweepHit.point, ref moveHitStabilityReport);
						Vector3 obstructionNormal = GetObstructionNormal (closestSweepHit.normal, moveHitStabilityReport);
						HitStatus.FoundAnyHit = true;
						HitStatus.HitCollider = closestSweepHit.collider;
						HitStatus.HitNormal = obstructionNormal;
						HitStatus.HitPoint = closestSweepHit.point;


						// Handle remembering rigidbody hits
						if (InteractiveRigidbodyHandling && closestSweepHit.collider.attachedRigidbody) {
							StoreRigidbodyHit (
								closestSweepHit.collider.attachedRigidbody, 
								(remainingMovementDirection * resultingMovementMagnitude) / deltaTime,
								closestSweepHit.point,
								obstructionNormal,
								moveHitStabilityReport);
						}


							// Project movement
							InternalHandleMovementProjection (
								moveHitStabilityReport.IsStable && !MustUnground,
								closestSweepHit.normal,
								obstructionNormal,
								originalMoveDirection,
								ref sweepState,
								ref previousMovementHitNormal,
								ref resultingMovementMagnitude,
								ref remainingMovementDirection,
								ref remainingMovementMagnitude);
						
					} 
                }
                // If we hit nothing...
                else
                {
                    hitSomethingThisSweepIteration = false;
                }

                // Safety for exceeding max sweeps allowed
                sweepsMade++;
                if (sweepsMade > MaxMovementSweepIterations)
                {
                    remainingMovementMagnitude = 0;
                    wasCompleted = false;
                }
            }
				
            // Move position for the remainder of the movement
            Vector3 targetFinalPosition = tmpMovedPosition + (remainingMovementDirection * remainingMovementMagnitude);
            InternalMoveCharacterPosition(ref _internalTransientPosition, targetFinalPosition, TransientRotation);
            resultingMovementDirection = remainingMovementDirection;

            return wasCompleted;
        }

        /// <summary>
        /// Gets the effective normal for movement obstruction depending on current grounding status
        /// </summary>
        private Vector3 GetObstructionNormal(Vector3 hitNormal, HitStabilityReport hitStabilityReport)
        {
            // Find hit/obstruction/offset normal
            Vector3 obstructionNormal = hitNormal;
            if (GroundingStatus.IsStableOnGround && !MustUnground && !hitStabilityReport.IsStable)
            {
                Vector3 obstructionLeftAlongGround = Vector3.Cross(GroundingStatus.GroundNormal, obstructionNormal).normalized;
                obstructionNormal = Vector3.Cross(obstructionLeftAlongGround, CharacterUp).normalized;
            }

            // Catch cases where cross product between parallel normals returned 0
            if(obstructionNormal == Vector3.zero)
            {
                obstructionNormal = hitNormal;
            }

            return obstructionNormal;
        }

        /// <summary>
        /// Remembers a rigidbody hit for processing later
        /// </summary>
        private void StoreRigidbodyHit(Rigidbody2D hitRigidbody, Vector2 hitVelocity, Vector2 hitPoint, Vector2 obstructionNormal, HitStabilityReport hitStabilityReport)
        {
            if (_rigidbodyProjectionHitCount < _internalRigidbodyProjectionHits.Length)
            {
                if (!hitRigidbody.GetComponent<CharacterController2D>())
                {
                    RigidbodyProjectionHit rph = new RigidbodyProjectionHit();
                    rph.Rigidbody = hitRigidbody;
                    rph.HitPoint = hitPoint;
                    rph.EffectiveHitNormal = obstructionNormal;
                    rph.HitVelocity = hitVelocity;
                    rph.StableOnHit = hitStabilityReport.IsStable;

                    _internalRigidbodyProjectionHits[_rigidbodyProjectionHitCount] = rph;
                    _rigidbodyProjectionHitCount++;
                }
            }
        }

        /// <summary>
        /// Processes movement projection upon detecting a hit
        /// </summary>
		private void InternalHandleMovementProjection(bool stableOnHit, Vector2 hitNormal, Vector2 obstructionNormal, Vector2 originalMoveDirection, ref MovementSweepState sweepState, 
			ref Vector2 previousObstructionNormal, ref float resultingMovementMagnitude, ref Vector2 remainingMovementDirection, ref float remainingMovementMagnitude)
        {
			if (remainingMovementMagnitude <= 0)
            {
                return;
            }

            Vector2 remainingMovement = originalMoveDirection * remainingMovementMagnitude;
            float remainingMagnitudeBeforeProj = remainingMovementMagnitude;
			if (stableOnHit)
            {
                LastMovementIterationFoundAnyGround = true;
            }


            // Blocking-corner handling
            if (sweepState == MovementSweepState.FoundBlockingCrease)
            {
                remainingMovementMagnitude = 0f;
                resultingMovementMagnitude = 0f;
                
                sweepState = MovementSweepState.FoundBlockingCorner;
            }
            // Handle projection
            else
            {
                CharacterController.HandleMovementProjection(ref remainingMovement, obstructionNormal, stableOnHit);

               remainingMovementDirection = remainingMovement.normalized;
                remainingMovementMagnitude = remainingMovement.magnitude;
                resultingMovementMagnitude = (remainingMovementMagnitude / remainingMagnitudeBeforeProj) * resultingMovementMagnitude;

                // Blocking corner handling
                if (sweepState == MovementSweepState.Initial)
                {
                    sweepState = MovementSweepState.AfterFirstHit;
                }
                else if (sweepState == MovementSweepState.AfterFirstHit)
                {
                    // Detect blocking corners
                    if (Vector3.Dot(previousObstructionNormal, remainingMovementDirection) < 0f)
                    {
                        Vector3 cornerVector = Vector3.Cross(previousObstructionNormal, obstructionNormal).normalized;
                        remainingMovement = Vector3.Project(remainingMovement, cornerVector);
                        remainingMovementDirection = remainingMovement.normalized;
                        remainingMovementMagnitude = remainingMovement.magnitude;
                        resultingMovementMagnitude = (remainingMovementMagnitude / remainingMagnitudeBeforeProj) * resultingMovementMagnitude;

                        sweepState = MovementSweepState.FoundBlockingCrease;
                    }
                }
            }

            previousObstructionNormal = obstructionNormal;
        }

        /// <summary>
        /// Moves the input position to the target. If SafeMovement is on, only move if we detect that the 
        /// character would not be overlapping with anything at the target position
        /// </summary>
        /// <returns> Returns true if no overlaps were found </returns>
        private bool InternalMoveCharacterPosition(ref Vector2 movedPosition, Vector2 targetPosition, Quaternion atRotation)
        {
            bool movementValid = true;
            if (SafeMovement)
            {
                int nbOverlaps = CharacterCollisionsOverlap(targetPosition, atRotation, _internalProbedColliders);
                if (nbOverlaps > 0)
                {
                    movementValid = false;
                }
            }

            if(movementValid)
            {
                movedPosition = targetPosition;
                return true;
            }
            
            return false;
        }

        /// <summary>
        /// Takes into account rigidbody hits for adding to the velocity
        /// </summary>
        private void ProcessVelocityForRigidbodyHits(ref Vector2 processedVelocity, float deltaTime)
        {
            for (int i = 0; i < _rigidbodyProjectionHitCount; i++)
            {
                if (_internalRigidbodyProjectionHits[i].Rigidbody)
                {
                    // Keep track of the unique rigidbodies we pushed this update, to avoid doubling their effect
                    bool alreadyPushedThisRigidbody = false;
                    for (int j = 0; j < _rigidbodiesPushedCount; j++)
                    {
                        if (_rigidbodiesPushedThisMove[j] == _internalRigidbodyProjectionHits[j].Rigidbody)
                        {
                            alreadyPushedThisRigidbody = true;
                            break;
                        }
                    }

                    if (!alreadyPushedThisRigidbody && _internalRigidbodyProjectionHits[i].Rigidbody != AttachedRigidbody)
                    {
                        if (_rigidbodiesPushedCount < _rigidbodiesPushedThisMove.Length)
                        {
                            // Remember we hit this rigidbody
                            _rigidbodiesPushedThisMove[_rigidbodiesPushedCount] = _internalRigidbodyProjectionHits[i].Rigidbody;
                            _rigidbodiesPushedCount++;

                            if(RigidbodyInteractionType == RigidbodyInteractionType.SimulatedDynamic)
                            {
                                CharacterController.HandleSimulatedRigidbodyInteraction(ref processedVelocity, _internalRigidbodyProjectionHits[i], deltaTime);
                            }                            
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Determines if the input collider is valid for collision processing
        /// </summary>
        /// <returns> Returns true if the collider is valid </returns>
		private bool CheckIfColliderValidForCollisions(Collider2D coll, bool validFromGround = false)
        {
            // Ignore self
            if (coll == null ||
                coll == Capsule)
            {
                return false;
            }

			if (!IsColliderValidForCollisions(coll, validFromGround))
            {
                return false;
            }

            return true;
        }

        /// <summary>
        /// Determines if the input collider is valid for collision processing
        /// </summary>
		private bool IsColliderValidForCollisions(Collider2D coll,  bool validFromGround = false)
        {
            // Ignore dynamic rigidbodies if the movement is made from AttachedRigidbody, or if RigidbodyInteractionType is kinematic
			if (!validFromGround && (_isMovingFromAttachedRigidbody || RigidbodyInteractionType == RigidbodyInteractionType.Kinematic) && coll.attachedRigidbody && !coll.attachedRigidbody.isKinematic)
            {
				return false;
            }

            // If movement is made from AttachedRigidbody, ignore the AttachedRigidbody
            if (_isMovingFromAttachedRigidbody && coll.attachedRigidbody == AttachedRigidbody)
            {
                return false;
            }

			if (coll.isTrigger) {
				return false;
			}
				
            // Custom checks
            if (!this.CharacterController.IsColliderValidForCollisions(coll))
            {
                return false;
            }

            return true;
        }

        /// <summary>
        /// Determines if the motor is considered stable on a given hit
        /// </summary>
        public void EvaluateHitStability(Collider2D hitCollider, Vector2 hitNormal, Vector2 hitPoint, Vector2 atCharacterPosition, Quaternion atCharacterRotation, ref HitStabilityReport stabilityReport)
        {
            if(!_solveGrounding)
            {
                stabilityReport.IsStable = false;
                return;
            }

            bool isStableOnNormal = false;
            Vector2 atCharacterUp = atCharacterRotation * Vector2.up;
            Vector2 innerHitDirection = Vector3.ProjectOnPlane(hitNormal, atCharacterUp).normalized;

            isStableOnNormal = this.IsStableOnNormal(hitNormal);
            stabilityReport.InnerNormal = hitNormal;
            stabilityReport.OuterNormal = hitNormal;
            
            // Step handling
            if (StepHandling != StepHandlingMethod.None && !isStableOnNormal)
            {
                // Stepping not supported on dynamic rigidbodies
                Rigidbody2D hitRigidbody = hitCollider.attachedRigidbody;
                if (!(hitRigidbody && !hitRigidbody.isKinematic))
                {
                    DetectSteps(atCharacterPosition, atCharacterRotation, hitPoint, innerHitDirection, ref stabilityReport);
                }
            }
            
            // Ledge handling
            if (LedgeHandling)
            {
                float ledgeCheckHeight = MinDistanceForLedge;
                if(StepHandling != StepHandlingMethod.None)
                {
                    ledgeCheckHeight = MaxStepHeight;
                }

                bool isStableLedgeInner = false;
                bool isStableLedgeOuter = false;

                RaycastHit2D innerLedgeHit;
                if (CharacterCollisionsRaycast(
                    hitPoint + (atCharacterUp * SecondaryProbesVertical) + (innerHitDirection * SecondaryProbesHorizontal), 
                    -atCharacterUp,
                    ledgeCheckHeight + SecondaryProbesVertical, 
                    out innerLedgeHit, 
                    _internalCharacterHits) > 0)
                {
                    stabilityReport.InnerNormal = innerLedgeHit.normal;
                    isStableLedgeInner = IsStableOnNormal(innerLedgeHit.normal);
                }

                RaycastHit2D outerLedgeHit;
                if (CharacterCollisionsRaycast(
                    hitPoint + (atCharacterUp * SecondaryProbesVertical) + (-innerHitDirection * SecondaryProbesHorizontal), 
                    -atCharacterUp,
                    ledgeCheckHeight + SecondaryProbesVertical, 
                    out outerLedgeHit, 
                    _internalCharacterHits) > 0)
                {
                    stabilityReport.OuterNormal = outerLedgeHit.normal;
                    isStableLedgeOuter = IsStableOnNormal(outerLedgeHit.normal);
                }
                
                stabilityReport.LedgeDetected = (isStableLedgeInner != isStableLedgeOuter);
                if (stabilityReport.LedgeDetected)
                {
                    stabilityReport.IsOnEmptySideOfLedge = isStableLedgeOuter && !isStableLedgeInner;
                    stabilityReport.LedgeGroundNormal = isStableLedgeOuter ? outerLedgeHit.normal : innerLedgeHit.normal;
                    stabilityReport.LedgeRightDirection = Vector3.Cross(hitNormal, outerLedgeHit.normal).normalized;
                    stabilityReport.LedgeFacingDirection = Vector3.Cross(stabilityReport.LedgeGroundNormal, stabilityReport.LedgeRightDirection).normalized;

					Vector2 bt = atCharacterRotation * CharacterTransformToCapsuleBottom;
					stabilityReport.DistanceFromLedge = Vector3.ProjectOnPlane((hitPoint - (atCharacterPosition + (bt))), atCharacterUp).magnitude;
                }
            }

            // Final stability evaluation
            if (isStableOnNormal || stabilityReport.ValidStepDetected)
            {
                stabilityReport.IsStable = true;
            }
				
            CharacterController.ProcessHitStabilityReport(hitCollider, hitNormal, hitPoint, atCharacterPosition, atCharacterRotation, ref stabilityReport);
        }

        private void DetectSteps(Vector2 characterPosition, Quaternion characterRotation, Vector2 hitPoint, Vector2 innerHitDirection, ref HitStabilityReport stabilityReport)
        {
            int nbStepHits = 0;
            Collider2D tmpCollider;
            RaycastHit2D outerStepHit;
            Vector2 characterUp = characterRotation * Vector2.up;
            Vector2 stepCheckStartPos = characterPosition;
            
            // Do outer step check with capsule cast on hit point
			stepCheckStartPos = characterPosition + (characterUp * MaxStepHeight) + (-innerHitDirection * CapsuleWidth);
            nbStepHits = CharacterCollisionsSweep(
                        stepCheckStartPos,
                        characterRotation,
                        -characterUp,
                        MaxStepHeight - CollisionOffset,
                        out outerStepHit,
                        _internalCharacterHits);

            // Check for overlaps and obstructions at the hit position
            if(CheckStepValidity(nbStepHits, characterPosition, characterRotation, innerHitDirection, stepCheckStartPos, out tmpCollider))
            {
                stabilityReport.ValidStepDetected = true;
                stabilityReport.SteppedCollider = tmpCollider;
            }

            if (StepHandling == StepHandlingMethod.Extra && !stabilityReport.ValidStepDetected)
            {
                // Do min reach step check with capsule cast on hit point
                stepCheckStartPos = characterPosition + (characterUp * MaxStepHeight) + (-innerHitDirection * MinRequiredStepDepth);
                nbStepHits = CharacterCollisionsSweep(
                            stepCheckStartPos,
                            characterRotation,
                            -characterUp,
                            MaxStepHeight - CollisionOffset,
                            out outerStepHit,
                            _internalCharacterHits);

                // Check for overlaps and obstructions at the hit position
                if (CheckStepValidity(nbStepHits, characterPosition, characterRotation, innerHitDirection, stepCheckStartPos, out tmpCollider))
                {
                    stabilityReport.ValidStepDetected = true;
                    stabilityReport.SteppedCollider = tmpCollider;
                }
            }
        }

        private bool CheckStepValidity(int nbStepHits, Vector2 characterPosition, Quaternion characterRotation, Vector2 innerHitDirection, Vector2 stepCheckStartPos, out Collider2D hitCollider)
        {
            hitCollider = null;
            Vector2 characterUp = characterRotation * Vector2.up;

            // Find the farthest valid hit for stepping
            bool foundValidStepPosition = false;
            while (nbStepHits > 0 && !foundValidStepPosition)
            {
                // Get farthest hit among the remaining hits
                RaycastHit2D farthestHit = new RaycastHit2D();
                float farthestDistance = 0f;
                int farthestIndex = 0;
                for (int i = 0; i < nbStepHits; i++)
                {
                    if (_internalCharacterHits[i].distance > farthestDistance)
                    {
                        farthestDistance = _internalCharacterHits[i].distance;
                        farthestHit = _internalCharacterHits[i];
                        farthestIndex = i;
                    }
                }
				Vector2  bottom = (characterRotation * CharacterTransformToCapsuleBottom);
				Vector2 characterBottom = characterPosition + bottom;
                float hitHeight = Vector3.Project(farthestHit.point - characterBottom, characterUp).magnitude;

                Vector2 characterPositionAtHit = stepCheckStartPos + (-characterUp * (farthestHit.distance - CollisionOffset));

                if (hitHeight <= MaxStepHeight)
                {
                    int atStepOverlaps = CharacterCollisionsOverlap(characterPositionAtHit, characterRotation, _internalProbedColliders);
                    if (atStepOverlaps <= 0)
                    {
                        // Check for outer hit slope normal stability at the step position
                        RaycastHit2D outerSlopeHit;
                        if (CharacterCollisionsRaycast(
                                farthestHit.point + (characterUp * SecondaryProbesVertical) + (-innerHitDirection * SecondaryProbesHorizontal),
                                -characterUp,
                                MaxStepHeight + SecondaryProbesVertical,
                                out outerSlopeHit,
                                _internalCharacterHits) > 0)
                        {
                            if (IsStableOnNormal(outerSlopeHit.normal))
                            {
                                // Cast upward to detect any obstructions to moving there
                                RaycastHit2D tmpUpObstructionHit;
                                if (CharacterCollisionsSweep(
                                                    characterPosition, // position
                                                    characterRotation, // rotation
                                                    characterUp, // direction
                                                    MaxStepHeight - farthestHit.distance, // distance
                                                    out tmpUpObstructionHit, // closest hit
                                                    _internalCharacterHits) // all hits
                                        <= 0)
                                {
                                    // Do inner step check...
                                    bool innerStepValid = false;
                                    RaycastHit2D innerStepHit;

									Vector2 prj = Vector3.Project ((characterPositionAtHit - characterPosition), characterUp);
                                    // At the capsule center at the step height
                                    if (CharacterCollisionsRaycast(
										characterPosition + prj,
                                        -characterUp,
                                        MaxStepHeight,
                                        out innerStepHit,
                                        _internalCharacterHits) > 0)
                                    {
                                        if (IsStableOnNormal(innerStepHit.normal))
                                        {
                                            innerStepValid = true;
                                        }
                                    }

                                    if (!innerStepValid)
                                    {
                                        // At inner step of the step point
                                        if (CharacterCollisionsRaycast(
                                            farthestHit.point + (innerHitDirection * SecondaryProbesHorizontal),
                                            -characterUp,
                                            MaxStepHeight,
                                            out innerStepHit,
                                            _internalCharacterHits) > 0)
                                        {
                                            if (IsStableOnNormal(innerStepHit.normal))
                                            {
                                                innerStepValid = true;
                                            }
                                        }
                                    }

                                    if (!innerStepValid)
                                    {
                                        // At the current ground point at the step height
                                    }

                                    // Final validation of step
                                    if (innerStepValid)
                                    {
                                        hitCollider = farthestHit.collider;
                                        foundValidStepPosition = true;
                                        return true;
                                    }
                                }
                            }
                        }
                    }
                }

                // Discard hit if not valid step
                if (!foundValidStepPosition)
                {
                    nbStepHits--;
                    if (farthestIndex < nbStepHits)
                    {
                        _internalCharacterHits[farthestIndex] = _internalCharacterHits[nbStepHits];
                    }
                }
            }
            
            return false;
        }

        /// <summary>
        /// Get true linear velocity (taking into account rotational velocity) on a given point of a rigidbody
        /// </summary>
        public Vector2 GetVelocityFromRigidbodyMovement(Rigidbody2D interactiveRigidbody, Vector2 atPoint, float deltaTime)
        {
            if (deltaTime > 0f)
            {
                Vector2 effectiveMoverVelocity = interactiveRigidbody.velocity;

                if (interactiveRigidbody.velocity != Vector2.zero)
                {
                    Vector2 centerOfRotation = interactiveRigidbody.position + interactiveRigidbody.centerOfMass;

                    Vector2 centerOfRotationToPoint = atPoint - centerOfRotation;
					Quaternion rotationFromInteractiveRigidbody = Quaternion.Euler(Mathf.Rad2Deg * (interactiveRigidbody.angularVelocity * Vector3.forward) * deltaTime);
					//Quaternion rotationFromInteractiveRigidbody = Quaternion.Euler(0,0, interactiveRigidbody.rotation);
					Vector2 velRot = (rotationFromInteractiveRigidbody * centerOfRotationToPoint);
					Vector2 finalPointPosition = centerOfRotation + (velRot);
                    effectiveMoverVelocity += (finalPointPosition - atPoint) / deltaTime;
                }
                return effectiveMoverVelocity;
            }
            else
            {
                return Vector3.zero;
            }
        }

        /// <summary>
        /// Determines if a collider has an attached interactive rigidbody
        /// </summary>
        private Rigidbody2D GetInteractiveRigidbody(Collider2D onCollider)
        {
            if (onCollider.attachedRigidbody)
            {
                if (onCollider.attachedRigidbody.gameObject.GetComponent<PhysicsMover>())
                {
                    return onCollider.attachedRigidbody;
                }

                if (!onCollider.attachedRigidbody.isKinematic)
                {
                    return onCollider.attachedRigidbody;
                }
            }
            return null;
        }

        /// <summary>
        /// Calculates the velocity required to move the character to the target position over a specific deltaTime.
        /// Useful for when you wish to work with positions rather than velocities in the UpdateVelocity callback of BaseCharacterController
        /// </summary>
        public Vector2 GetVelocityForMovePosition(Vector3 fromPosition, Vector3 toPosition, float deltaTime)
        {
            if (deltaTime > 0)
            {
                return (toPosition - fromPosition) / deltaTime;
            }
            else
            {
                return Vector2.zero;
            }
        }

        /// <summary>
        /// Detect if the character capsule is overlapping with anything collidable
        /// </summary>
        /// <returns> Returns number of overlaps </returns>
		public int CharacterCollisionsOverlap(Vector2 atPosition, Quaternion atRotation, Collider2D[] overlappedColliders, float radiusInflate = 0f, bool verifyOneSide = true)
        {
            int nbHits = 0;

			int nbUnfilteredHits = Physics2D.OverlapCapsuleNonAlloc (
				atPosition + CharacterTransformToCapsuleCenter,
				Capsule.size + new Vector2(radiusInflate, 0),
				_capsuleDirection,
				atRotation.z,
				overlappedColliders,
				CollidableLayers);


            // Filter out invalid colliders
            nbHits = nbUnfilteredHits;
            for (int i = nbUnfilteredHits - 1; i >= 0; i--)
            {
				if (!CheckIfColliderValidForCollisions(overlappedColliders[i]) || (overlappedColliders[i].transform.CompareTag("OneSide") && verifyOneSide))
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        overlappedColliders[i] = overlappedColliders[nbHits];
                    }
                }
            }

            return nbHits;
        }

        /// <summary>
        /// Detect if the character capsule is overlapping with anything
        /// </summary>
        /// <returns> Returns number of overlaps </returns>
        public int CharacterOverlap(Vector2 atPosition, Quaternion atRotation, Collider2D[] overlappedColliders, LayerMask layers, QueryTriggerInteraction triggerInteraction, float radiusInflate = 0f)
        {
            int nbHits = 0;

			int nbUnfilteredHits = Physics2D.OverlapCapsuleNonAlloc (
				atPosition + CharacterTransformToCapsuleCenter,
				Capsule.size + new Vector2(radiusInflate, 0),
				_capsuleDirection,
				atRotation.z,
				overlappedColliders,
				CollidableLayers);


            // Filter out the character capsule itself
            nbHits = nbUnfilteredHits;
            for (int i = nbUnfilteredHits - 1; i >= 0; i--)
            {
                if (overlappedColliders[i] == Capsule)
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        overlappedColliders[i] = overlappedColliders[nbHits];
                    }
                }
            }

            return nbHits;
        }

		public bool isUp()
		{
			float yMovement = Mathf.Floor(_lastPosition.y) - Mathf.Floor(TransientPosition.y);
			return yMovement > 0;
		}

		private bool AllowOneSide(Vector2 position, Quaternion rotation, Collider2D hitCollider, Vector2 hitPoint, Vector2 normal, float distance = OneSideGroundProbingDistance)
		{
			Transform hitTrasform = hitCollider.transform;

			Vector2 bottom = rotation * CharacterTransformToCapsuleBottom;
			//Debug.DrawLine (hitPoint, hitPoint + new Vector2(0.1f, 0.1f));
			bool result = ((hitTrasform.CompareTag ("OneSide")) && ((hitPoint.y - distance) > ((position.y + bottom.y)))) == false;

			return result;
		} 

		public bool IsSnappedToGround()
		{
			return _isSnappingToGround;
		}

		public bool IsOneSideGround(Collider2D collider)
		{
			if (collider != null && collider.transform.CompareTag ("OneSide")) {
				return true;
			}

			return false;
		}

        /// <summary>
        /// Sweeps the capsule's volume to detect collision hits
        /// </summary>
        /// <returns> Returns the number of hits </returns>
		public int CharacterCollisionsSweep(Vector2 position,  Quaternion rotation, Vector2 direction, float distance, out RaycastHit2D closestHit, RaycastHit2D[] hits, float radiusInflate = 0f, float capsuleHeight = 0)
        {
            direction.Normalize();

			Vector2 size = Capsule.size;
			if (capsuleHeight != 0) {
				size.y = capsuleHeight;
			}
            
            // Capsule cast
            int nbHits = 0;

			int nbUnfilteredHits = Physics2D.CapsuleCastNonAlloc (
				position + CharacterTransformToCapsuleCenter ,
				size + new Vector2(radiusInflate, -SweepProbingBackstepDistance),
				_capsuleDirection,
				rotation.z,
				direction,  
				hits,
				distance + SweepProbingBackstepDistance,
				CollidableLayers);

            // Hits filter
            closestHit = new RaycastHit2D();
            float closestDistance = Mathf.Infinity;
            nbHits = nbUnfilteredHits;
			Vector2 bottom = rotation * CharacterTransformToCapsuleBottom;
            for (int i = nbUnfilteredHits - 1; i >= 0; i--)
            {
                hits[i].distance -= SweepProbingBackstepDistance;
                // Filter out the invalid hits
                if (hits[i].distance <= 0f ||
					!CheckIfColliderValidForCollisions(hits[i].collider, ((hits [i].point.y - GroundProbingBackstepDistance) < position.y + bottom.y))
					|| !AllowOneSide(position,rotation, hits [i].collider, hits [i].point, hits [i].normal))
                {
					
						nbHits--;
						if (i < nbHits) {
							hits [i] = hits [nbHits];
						}

                }
                else
                {
					
						// Remember closest valid hit
						if (hits [i].distance < closestDistance) {
							closestHit = hits [i];
							closestDistance = hits [i].distance;
						}

                }
            }

            return nbHits;
        }

        /// <summary>
        /// Sweeps the capsule's volume to detect hits
        /// </summary>
        /// <returns> Returns the number of hits </returns>
        public int CharacterSweep(Vector2 position, Quaternion rotation, Vector2 direction, float distance, out RaycastHit2D closestHit, RaycastHit2D[] hits, LayerMask layers, QueryTriggerInteraction triggerInteraction, float radiusInflate = 0f)
        {
            direction.Normalize();
            closestHit = new RaycastHit2D();

            // Capsule cast
            int nbHits = 0;
			int nbUnfilteredHits = Physics2D.CapsuleCastNonAlloc (
				position + CharacterTransformToCapsuleCenter,
				Capsule.size + new Vector2(radiusInflate, 0),
				_capsuleDirection,
				rotation.z,
				direction,  
				hits,
				distance,
				CollidableLayers);

            // Hits filter
            float closestDistance = Mathf.Infinity;
            nbHits = nbUnfilteredHits;
            for (int i = nbUnfilteredHits - 1; i >= 0; i--)
            {
                // Filter out the character capsule
                if (hits[i].distance <= 0f || hits[i].collider == Capsule)
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        hits[i] = hits[nbHits];
                    }
                }
                else
                {
                    // Remember closest valid hit
                    if (hits[i].distance < closestDistance)
                    {
                        closestHit = hits[i];
                        closestDistance = hits[i].distance;
                    }
                }
            }

            return nbHits;
        }

        /// <summary>
        /// Casts the character volume in the character's downward direction to detect ground
        /// </summary>
        /// <returns> Returns the number of hits </returns>
        private bool CharacterGroundSweep(Vector2 position, float angle, Vector2 direction, float distance, out RaycastHit2D closestHit)
        {
			Quaternion rot = Quaternion.Euler(new Vector3(0,0, angle));
				
            direction.Normalize();
            closestHit = new RaycastHit2D();

			int nbUnfilteredHits = Physics2D.CapsuleCastNonAlloc (
				position + CharacterTransformToCapsuleCenter ,
				Capsule.size  + new Vector2(0,  -GroundProbingBackstepDistance),
				_capsuleDirection,
				angle,
				direction,  
				_internalCharacterHits,
				distance + GroundProbingBackstepDistance,
				CollidableLayers);
			

            // Hits filter
            bool foundValidHit = false;
            float closestDistance = Mathf.Infinity;

            for (int i = 0; i < nbUnfilteredHits; i++)
            {
                // Find the closest valid hit
				if (_internalCharacterHits[i].distance > 0f && (CheckIfColliderValidForCollisions(_internalCharacterHits[i].collider, true)
					&& AllowOneSide(position, rot, _internalCharacterHits [i].collider, _internalCharacterHits [i].point,_internalCharacterHits [i].normal)))
                {
					//Debug.DrawLine (position + CharacterTransformToCapsuleBottom, position + CharacterTransformToCapsuleBottom + new Vector2(0.1f, 0.1f));

						if (_internalCharacterHits [i].distance < closestDistance) {
							closestHit = _internalCharacterHits [i];
							closestHit.distance -= GroundProbingBackstepDistance;
							closestDistance = _internalCharacterHits [i].distance;

							foundValidHit = true;
						}

                }
            }

            return foundValidHit;
        }

        /// <summary>
        /// Raycasts to detect collision hits
        /// </summary>
        /// <returns> Returns the number of hits </returns>
		public int CharacterCollisionsRaycast(Vector2 position, Vector2 direction, float distance, out RaycastHit2D closestHit, RaycastHit2D[] hits)
        {
            direction.Normalize();

            // Raycast
            int nbHits = 0;
			int nbUnfilteredHits = Physics2D.RaycastNonAlloc(
				position,
				direction,
				hits,
				distance,
				CollidableLayers);

            // Hits filter
            closestHit = new RaycastHit2D();
            float closestDistance = Mathf.Infinity;
            nbHits = nbUnfilteredHits;
            for (int i = nbUnfilteredHits - 1; i >= 0; i--)
            {
                // Filter out the invalid hits
                if (hits[i].distance <= 0f ||
                    !CheckIfColliderValidForCollisions(hits[i].collider)
					|| !AllowOneSide(position, TransientRotation,  hits [i].collider, hits [i].point, hits [i].normal))
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        hits[i] = hits[nbHits];
                    }
                }
                else
                {
                    // Remember closest valid hit
                    if (hits[i].distance < closestDistance)
                    {
                        closestHit = hits[i];
                        closestDistance = hits[i].distance;
                    }
                }
            }

            return nbHits;
        }
    }
}