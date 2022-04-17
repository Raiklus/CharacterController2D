using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public struct PlayerCharacterInputs
{
	public float MoveAxisForward;
	public float MoveAxisRight;
	public float LookAxisForward;
	public float LookAxisRight;
	public bool JumpDown;
	public Quaternion CameraRotation;
}