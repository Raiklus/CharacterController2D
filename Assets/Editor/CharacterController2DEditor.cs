using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace CooldHands
{
	[CustomEditor(typeof(CharacterController2D))]
	public class CharacterController2DEditor : Editor
    {
        protected virtual void OnSceneGUI()
        {            
			CharacterController2D motor = (target as CharacterController2D);
            if (motor)
            {
				Vector3 offset = (motor.Capsule.offset + (-Vector2.up * (motor.Capsule.size.y * 0.5f)));
				Vector3 characterBottom = motor.transform.position + offset;

                Handles.color = Color.yellow;
                Handles.CircleHandleCap(
                    0, 
                    characterBottom + (motor.transform.up * motor.MaxStepHeight), 
                    Quaternion.LookRotation(motor.transform.up, motor.transform.forward), 
                    motor.Capsule.size.x + 0.1f, 
                    EventType.Repaint);
            }
        }
    }
}