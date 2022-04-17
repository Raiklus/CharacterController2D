using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CooldHands;
using System;
using UnityEngine.Events;

public class WallSlideAction : BaseCharacterAction
{
	[Serializable]
	public class WallSlideActionEvent : UnityEvent<CharacterHitReport>
	{

	}

	Vector2 OriginalGravity;
	bool isExecuting = false;
	public float handsPosition = 0;
	public bool snapToWall = false;
	public float fallingSpeed = -2f;
	public LayerMask wallSlideLayers;

	private RaycastHit2D[] _internalCharacterHits;
	private Vector3 _moveAxis;
	private Vector3 _direction;
	private bool _verifyCollision = false;
	Vector2 _handPosCenterToBottom;
	float _capHeight;
	CharacterHitReport _hit = new CharacterHitReport ();

	public WallSlideActionEvent OnSlidingWall;

	void Start ()
	{
		OriginalGravity = Controller2D.Gravity;
		_internalCharacterHits = new RaycastHit2D[CharacterController2D.MaxHitsBudget];

		Vector3 handPos = Controller2D.CharacterTransformToCapsuleCenter + Vector2.up * handsPosition;
		_capHeight = handPos.y + Controller2D.CharacterTransformToCapsuleBottom.y;
	    _handPosCenterToBottom = ((handPos + Controller2D.CharacterTransformToCapsuleBottom) * 0.5f);
	}

	/// <summary>
	/// Execute action
	/// </summary>
	public override CharacterActionResult Execute (float deltaTime)
	{
		CharacterActionResult result = new CharacterActionResult ();
		result.velocity = Controller2D.BaseVelocity;
		_hit.FoundAnyHit = false;

		_verifyCollision = false;

		if (_moveAxis != Vector3.zero /*&& !snapToWall*/) {
			_direction = _moveAxis;
		}

		if (Controller2D.HitStatus.FoundAnyHit && !isExecuting) {
			if (((wallSlideLayers.value & 1 << Controller2D.HitStatus.HitCollider.gameObject.layer) == 1 << Controller2D.HitStatus.HitCollider.gameObject.layer)) {
				isExecuting = true;

				if (snapToWall) {
					_direction = (Controller2D.HitStatus.HitPoint - Controller2D.TransientPosition);
				}
			}
		} 

		if (snapToWall || _moveAxis != Vector3.zero) {
			_verifyCollision = true;
		}
			
		if (isExecuting && _verifyCollision) {
			Vector2 targetMovementVelocity = Vector2.zero;
			RaycastHit2D closestSweepHit;
				
			if (Controller2D.CharacterCollisionsSweep (Controller2D.TransientPosition + _handPosCenterToBottom - Controller2D.CharacterTransformToCapsuleCenter, Controller2D.TransientRotation, _direction, 0.2f, out closestSweepHit, _internalCharacterHits, 0, _capHeight) > 0) {
				if ((!Controller2D.MustUnground)) {
					Controller2D.Gravity = Vector2.zero;

					if (snapToWall) 
					{
						_direction = (closestSweepHit.point - Controller2D.TransientPosition);
					}

					result.velocity = (-Controller2D.CharacterUp * fallingSpeed) + (Controller2D.CharacterRight * result.velocity.x);

					_hit.FoundAnyHit = true;
					_hit.HitCollider = closestSweepHit.collider;
					_hit.HitNormal = closestSweepHit.normal;
					_hit.HitPoint = closestSweepHit.point;

					result.velocity = Vector3.ProjectOnPlane (result.velocity, _hit.HitNormal);

					targetMovementVelocity = result.velocity;
				
					// Smooth movement Velocity
					result.velocity = Vector2.Lerp (result.velocity, targetMovementVelocity, deltaTime);

					OnSlidingWall.Invoke (_hit);

				} else {
					if (_moveAxis != Vector3.zero) {
						_direction = _moveAxis;
					}
					Controller2D.Gravity = OriginalGravity;
				}
			} else {
				ExitExecute (deltaTime);
			}

		} else {
			ExitExecute (deltaTime);
		}
		return result;
	}

	/// <summary>
	/// Wait to action exit
	/// </summary>
	/// <returns>The to exit.</returns>
	IEnumerator WaitToExit()
	{
		yield return new WaitForSeconds (0.15f);
		OnSlidingWall.Invoke (_hit);

	}

	/// <summary>
	/// Exit action
	/// </summary>
	public override CharacterActionResult ExitExecute (float deltaTime)
	{
		if (isExecuting) {
			isExecuting = false;
			StartCoroutine (WaitToExit ());
			Controller2D.Gravity = OriginalGravity;
		}
		return null;
	}

	/// <summary>
	/// Prepare action to execute
	/// </summary>
	public override void RequestAction (params object[] parameters)
	{
		_moveAxis = (Vector3)parameters [0];
		base.RequestAction (parameters);
	}

	/// <summary>
	/// Raises the draw gizmos event.
	/// </summary>
	void OnDrawGizmosSelected ()
	{
		Color col = Gizmos.color;
		Gizmos.color = Color.cyan;

		Vector3 widthHalf = Vector3.right * ((Controller2D.Capsule.size.x * 0.5f) + 1f);

		Vector3 center = Controller2D.Capsule.offset + Vector2.up * handsPosition;

		//Vector3 bttom = Controller2D.Capsule.offset + (-Vector2.up * (Controller2D.Capsule.size.y * 0.5f));
		//Vector3 posCenter = (center + bttom) * 0.5f;

		Gizmos.DrawLine (this.transform.position + center - widthHalf ,this.transform.position + center + widthHalf);
		//Gizmos.DrawLine (this.transform.position + bttom - widthHalf ,this.transform.position + bttom + widthHalf);
		//Gizmos.DrawLine (this.transform.position + posCenter - widthHalf,this.transform.position + posCenter + widthHalf);
		Gizmos.color = col;
	}
}
