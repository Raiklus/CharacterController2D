using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CooldHands;

public class JumpAction : BaseCharacterAction
{

	[Header ("Jumping")]
	public bool AllowJumpingWhenSliding = false;
	public bool AllowDoubleJump = false;
	public bool AllowWallJump = false;
	public float JumpSpeed = 10f;

	private bool _jumpRequested = false;
	private bool _doubleJumpConsumed = false;
	private bool _jumpedThisFrame = false;
	private bool _canWallJump = false;
	private Vector3 _wallJumpNormal;
//	private Vector3 _moveAxis;
	private bool _foundWall= false;

	/// <summary>
	/// Execute action
	/// </summary>
	public override CharacterActionResult Execute (float deltaTime)
	{
		CharacterActionResult result = new CharacterActionResult ();
		result.velocity = Controller2D.BaseVelocity;

		// Handle jumping
		_jumpedThisFrame = false;

		if (_jumpRequested) {
			if (AllowDoubleJump) {
				// See if we actually are allowed to jump
				if ((!_doubleJumpConsumed && !_canWallJump && !_foundWall && ((AllowJumpingWhenSliding ? (!Controller2D.GroundingStatus.FoundAnyGround) : !Controller2D.GroundingStatus.IsStableOnGround)))) {
					// Calculate jump direction before ungrounding
					Vector3 jumpDirection = Controller2D.CharacterUp;
					if (Controller2D.GroundingStatus.FoundAnyGround && !Controller2D.GroundingStatus.IsStableOnGround) {
						//jumpDirection = Controller2D.GroundingStatus.GroundNormal;
					}
					
					// Makes the character skip ground probing/snapping on its next update. 
					// If this line weren't here, the character would remain snapped to the ground when trying to jump. Try commenting this line out and see.
					Controller2D.ForceUnground ();

					// Add to the return velocity and reset jump state
					Vector2 jum = (jumpDirection * JumpSpeed) - Vector3.Project (result.velocity, Controller2D.CharacterUp);
					result.velocity = result.velocity + jum;
					_jumpRequested = false;
					_doubleJumpConsumed = true;
					_jumpedThisFrame = true;
				}
			}


			// See if we actually are allowed to jump
			if (_canWallJump ||
				(((AllowJumpingWhenSliding ? Controller2D.GroundingStatus.FoundAnyGround : Controller2D.GroundingStatus.IsStableOnGround)))) {
				// Calculate jump direction before ungrounding
				Vector3 jumpDirection = Controller2D.CharacterUp;
				Vector3 up = Controller2D.CharacterUp;

				if (_canWallJump) {
					jumpDirection = Vector3.ProjectOnPlane (_wallJumpNormal, Controller2D.CharacterUp) + up;

				} else if (Controller2D.GroundingStatus.FoundAnyGround && !Controller2D.GroundingStatus.IsStableOnGround) {
					//jumpDirection = Controller2D.GroundingStatus.GroundNormal;
				}

				// Makes the character skip ground probing/snapping on its next update. 
				// If this line weren't here, the character would remain snapped to the ground when trying to jump. Try commenting this line out and see.
				Controller2D.ForceUnground ();

				// Add to the return velocity and reset jump state
				Vector2 jum = (jumpDirection * JumpSpeed) - Vector3.Project (result.velocity, Controller2D.CharacterUp);
				//Vector2 vel = (_wallJumpNormal * 4);
				result.velocity += jum;
				_jumpRequested = false;
				_jumpedThisFrame = true;

			} 
				
		}


		if (AllowWallJump) {
			_canWallJump = _foundWall;
		} else {
			_canWallJump = false;
		}

		return result;
	}
		
	/// <summary>
	// Set if we can wall jump
	/// </summary>
	public void SetAllowWallJump (CharacterHitReport hit)
	{
		if (hit.FoundAnyHit) {
			_foundWall = true;
			if (AllowWallJump) {
				_canWallJump = true;
				_wallJumpNormal = hit.HitNormal;
				if (!_jumpedThisFrame) {
					_doubleJumpConsumed = false;
				}
			}
			//_doubleJumpConsumed = false;
			//_jumpConsumed = false;
		} else {
			_foundWall = false;
		}
	}

	/// <summary>
	/// Exit action
	/// </summary>
	public override CharacterActionResult ExitExecute (float deltaTime)
	{
		_jumpRequested = false;

		if (Controller2D.IsSnappedToGround() && (AllowJumpingWhenSliding ? (Controller2D.GroundingStatus.FoundAnyGround) : (Controller2D.GroundingStatus.IsStableOnGround))) {
			// If we're on a ground surface, reset jumping values
			if (!_jumpedThisFrame) {
				_doubleJumpConsumed = false;
				//_canWallJump = false;
			}
		} 

		return null;
	}

	/// <summary>
	/// Prepare action to execute
	/// </summary>
	public override void RequestAction (params object[] parameters)
	{
		base.RequestAction (parameters);
		_jumpRequested = true;
		//_moveAxis = (Vector3)parameters [0];
	}
}
