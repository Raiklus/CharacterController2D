using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CooldHands;
using System;
using UnityEngine.Events;


	public enum CharacterState
	{
		Default,
		WallSliding,
	}

	public class PlayerController : BaseCharacterController
	{
		[Serializable]
		public class PlayerControllerEvent : UnityEvent
		{

		}

		[Header ("Stable Movement")]
		public float OrientationSharpness = 10;

		[Header ("Misc")]
		public Transform MeshRoot;

		public CharacterState CurrentCharacterState { get; private set; }

		private Vector3 _moveInputVector;
//		private Vector3 _lookInputVector;
		public Animator _animator;

		public BaseCharacterAction MoveAction;
		public BaseCharacterAction JumpAction;
		public BaseCharacterAction SlideAction;


		void Start ()
		{
			// Handle initial state
			TransitionToState(CharacterState.Default);
		}

		/// <summary>
		/// Handles movement state transitions and enter/exit callbacks
		/// </summary>
		public void TransitionToState (CharacterState newState)
		{
			CharacterState tmpInitialState = CurrentCharacterState;
			OnStateExit (tmpInitialState, newState);
			CurrentCharacterState = newState;
			OnStateEnter (newState, tmpInitialState);
		}

		/// <summary>
		/// Event when entering a state
		/// </summary>
		public void OnStateEnter (CharacterState state, CharacterState fromState)
		{
			switch (state)
			{
			case CharacterState.Default:
				{
					break;
				}
			case CharacterState.WallSliding:
				{
					break;
				}
			}
		}

		/// <summary>
		/// Event when exiting a state
		/// </summary>
		public void OnStateExit (CharacterState state, CharacterState toState)
		{
			switch (state)
			{
				case CharacterState.Default:
				{
					break;
				}
			}
		}

		/// <summary>
		/// This is called every frame by MyPlayer in order to tell the character what its inputs are
		/// </summary>
		public void SetInputs (ref PlayerCharacterInputs inputs)
		{
			// Clamp input
			Vector3 moveInputVector = Vector3.ClampMagnitude (new Vector3 (inputs.MoveAxisRight, 0f, inputs.MoveAxisForward), 1f);



			// Calculate camera direction and rotation on the character plane
			Vector3 cameraPlanarDirection = Vector3.ProjectOnPlane (inputs.CameraRotation * Vector3.forward, Controller2D.CharacterUp).normalized;
			if (cameraPlanarDirection.sqrMagnitude == 0f) {
				cameraPlanarDirection = Vector3.ProjectOnPlane (inputs.CameraRotation * Vector3.up, Controller2D.CharacterUp).normalized;
			}
			Quaternion cameraPlanarRotation = Quaternion.LookRotation (cameraPlanarDirection, Controller2D.CharacterUp);
			//cameraPlanarRotation *= Quaternion.FromToRotation (Motor.CharacterUp, _zTerrainNormal);


			// Move and look inputs

			//_moveInputVector = Quaternion.FromToRotation (Motor.CharacterUp, Motor.GroundingStatus.GroundNormal) *  cameraPlanarRotation * moveInputVector;
			//_lookInputVector = _moveInputVector.normalized;
//			Vector3 lookInputVector = Vector3.ClampMagnitude (new Vector3 (inputs.LookAxisRight, 0f, inputs.LookAxisForward), 1f);
			_moveInputVector = cameraPlanarRotation * moveInputVector;
//			_lookInputVector = (cameraPlanarRotation * (lookInputVector));

			if (MoveAction != null) {
				MoveAction.RequestAction (_moveInputVector);
				
			}

			if (JumpAction != null && inputs.JumpDown) {
				JumpAction.RequestAction (_moveInputVector);
			}

			if (SlideAction != null) {
				SlideAction.RequestAction (_moveInputVector);
			}
		}

		public override void BeforeCharacterUpdate (float deltaTime)
		{
		}

		public override void UpdateRotation (ref Quaternion currentRotation, float deltaTime)
		{

			if ((_moveInputVector != Vector3.zero || Controller2D.GroundingStatus.FoundAnyGround) && OrientationSharpness > 0f && Controller2D.GroundingStatus.IsStableOnGround) {
				Vector2 effectiveGroundNormal = Controller2D.GroundingStatus.GroundNormal;
				if (Controller2D.Velocity.sqrMagnitude > 0f && Controller2D.GroundingStatus.SnappingPrevented) {
					// Take the normal from where we're coming from
					Vector2 groundPointToCharacter = Controller2D.TransientPosition - Controller2D.GroundingStatus.GroundPoint;
					if (Vector2.Dot (Controller2D.Velocity, groundPointToCharacter) >= 0f) {
						effectiveGroundNormal = Controller2D.GroundingStatus.OuterGroundNormal;
					} else {
						effectiveGroundNormal = Controller2D.GroundingStatus.InnerGroundNormal;
					}
				}


				Vector3 lookRot = Vector3.up;

				bool stableAngle = Vector2.Angle (Vector2.up, effectiveGroundNormal) <= Controller2D.MaxStableSlopeAngle;

				if (stableAngle) {
					//reorientedInput = Controller2D.GetDirectionTangentToSurface (lookVector, effectiveGroundNormal) * lookVector.magnitude;
					lookRot = effectiveGroundNormal;
				
					Quaternion rot = Quaternion.LookRotation (Vector3.forward, lookRot);
	
					//currentRotation = Quaternion.Lerp (Controller2D.TransientRotation, rot, OrientationSharpness * deltaTime);

				}
	
			} else {
				//currentRotation = Quaternion.Lerp (Controller2D.TransientRotation, Quaternion.identity, OrientationSharpness * deltaTime);
			}



		}

		public override void UpdateVelocity (ref Vector2 currentVelocity, float deltaTime)
		{
			if (MoveAction != null) {
				currentVelocity = MoveAction.Execute (deltaTime).velocity;
			}
			
			

			if (JumpAction != null) {
				currentVelocity = JumpAction.Execute (deltaTime).velocity;
			}

		if (SlideAction != null) {
			currentVelocity = SlideAction.Execute (deltaTime).velocity;
		}

			//Debug.Log (Controller2D.HitStatus.FoundAnyHit.ToString ());
		}

		public override void AfterCharacterUpdate (float deltaTime)
		{
			if (JumpAction != null) {
				JumpAction.ExitExecute (deltaTime);
			}
		}

		public override bool IsColliderValidForCollisions (Collider2D coll)
		{
			return true;
		}

		public override void OnGroundHit (Collider2D hitCollider, Vector2 hitNormal, Vector2 hitPoint, ref HitStabilityReport hitStabilityReport)
		{
			//Debug.Log ("hitGround");
			if (hitStabilityReport.LedgeDetected) {
				//Debug.Log ("ledge");
			}
		}

		public override void OnMovementHit (Collider2D hitCollider, Vector2 hitNormal, Vector2 hitPoint, ref HitStabilityReport hitStabilityReport)
		{
			
			if (hitStabilityReport.IsStable) {
				//Debug.Log ("IsStable");
			}
		}

		public override void ProcessHitStabilityReport (Collider2D hitCollider, Vector2 hitNormal, Vector2 hitPoint, Vector2 atCharacterPosition, Quaternion atCharacterRotation, ref HitStabilityReport hitStabilityReport)
		{
			if (hitStabilityReport.LedgeDetected) {
				//Debug.Log ("ledge");
			}
		}

		public override void PostGroundingUpdate (float deltaTime)
		{
			if (Controller2D.GroundingStatus.IsStableOnGround && !Controller2D.LastGroundingStatus.IsStableOnGround) {
				OnLanded ();
			} else if (!Controller2D.GroundingStatus.IsStableOnGround && Controller2D.LastGroundingStatus.IsStableOnGround) {
				OnLeaveStableGround ();
			}

			/*if (MeshRoot != null) {
				Vector3 effectiveGroundNormal = Motor.GroundingStatus.GroundNormal;

				// Calculate target velocity
				Vector3 inputRight = Vector3.Cross (_lookInputVector, MeshRoot.transform.up);
				Vector3 reorientedInput = Vector3.Cross (effectiveGroundNormal, inputRight).normalized * _lookInputVector.magnitude;

				Vector3 smoothedLookInputDirection2 = Vector3.Slerp (MeshRoot.transform.forward, reorientedInput, 1 - Mathf.Exp (-OrientationSharpness * deltaTime)).normalized;
				MeshRoot.rotation = Quaternion.LookRotation (smoothedLookInputDirection2, effectiveGroundNormal);
			}*/
		}

		protected void OnLanded ()
		{
			
			//Debug.Log ("landed");
		}

		protected void OnLeaveStableGround ()
		{
			//landed = false;
			//Debug.Log ("no..");
		}
	}
