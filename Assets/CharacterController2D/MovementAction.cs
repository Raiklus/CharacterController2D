using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CooldHands;

public class MovementAction : BaseCharacterAction {

	[Header ("Stable Movement")]
	public float MaxStableMoveSpeed = 10f;
	public float StableMovementSharpness = 15;

	[Header ("Air Movement")]
	public float MaxAirMoveSpeed = 10f;
	public float AirAccelerationSpeed = 5f;

	/// <summary>
	/// Execute action
	/// </summary>
	public override CharacterActionResult Execute(float deltaTime)
	{
		
		Vector3 _moveInputVector = Vector3.zero;

		if (Parameters != null) {
			_moveInputVector = (Vector3)Parameters [0];
		}

		CharacterActionResult result = new CharacterActionResult ();
		result.velocity = Controller2D.BaseVelocity;

		Vector2 targetMovementVelocity = Vector2.zero;
		if (Controller2D.GroundingStatus.IsStableOnGround) {

			Vector2 effectiveGroundNormal = Controller2D.GroundingStatus.GroundNormal;
			if (result.velocity.sqrMagnitude > 0f && Controller2D.GroundingStatus.SnappingPrevented) {
				// Take the normal from where we're coming from
				Vector2 groundPointToCharacter = Controller2D.TransientPosition - Controller2D.GroundingStatus.GroundPoint;
				if (Vector2.Dot (result.velocity, groundPointToCharacter) >= 0f) {
					effectiveGroundNormal = Controller2D.GroundingStatus.OuterGroundNormal;
				} else {
					effectiveGroundNormal = Controller2D.GroundingStatus.InnerGroundNormal;
				}
			}

			float velocityY = result.velocity.y;
			// Reorient velocity on slope
			if (Controller2D.IsSnappedToGround()) {
				result.velocity = Controller2D.GetDirectionTangentToSurface (result.velocity, effectiveGroundNormal) * result.velocity.magnitude;
			}


			// Calculate target velocity
			Vector3 inputRight = Vector3.Cross (_moveInputVector, Controller2D.CharacterUp);
			Vector3 reorientedInput = Vector3.Cross (effectiveGroundNormal, inputRight).normalized * _moveInputVector.magnitude;
			targetMovementVelocity = reorientedInput * MaxStableMoveSpeed;
			targetMovementVelocity.y = velocityY;

			// Smooth movement Velocity
			result.velocity = Vector2.Lerp (result.velocity, targetMovementVelocity,StableMovementSharpness * deltaTime);

		} else {

			// Add move input
			//if (_moveInputVector.sqrMagnitude > 0f) {
				targetMovementVelocity = _moveInputVector * MaxAirMoveSpeed;

				if (_moveInputVector.sqrMagnitude > 0f) {
					// Prevent climbing on un-stable slopes with air movement
					if (Controller2D.GroundingStatus.FoundAnyGround) {
						Vector3 perpenticularObstructionNormal = Vector3.Cross (Vector3.Cross (Controller2D.CharacterUp, Controller2D.GroundingStatus.GroundNormal), Controller2D.CharacterUp).normalized;
						targetMovementVelocity = Vector3.ProjectOnPlane (targetMovementVelocity, perpenticularObstructionNormal);
					}
				}
					
				Vector3 velocityDiff = Vector3.ProjectOnPlane (targetMovementVelocity - result.velocity, Controller2D.Gravity);
				result.velocity += new Vector2 (velocityDiff.x, velocityDiff.y) * AirAccelerationSpeed * deltaTime;
			//}
		}

		return result;
	}
}
