using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CooldHands;


namespace CooldHands
{

public class PlayerInputs : MonoBehaviour {

		public PlayerController Character;

		private const string HorizontalInput = "Horizontal";
		private const string VerticalInput = "Vertical";
	

		private void Start()
		{
			//Cursor.lockState = CursorLockMode.Locked;
			Application.targetFrameRate = 60;
		}
			

		private void Update()
		{
			//if (Input.GetMouseButtonDown(0))
			//{
			//Cursor.lockState = CursorLockMode.Locked;
			// }

			//if (Character != null && !Character._isDead) {
				HandleCameraInput ();
				HandleCharacterInput ();
				//Shot ();
			//}
		}
			

		private void HandleCharacterInput()
		{
			PlayerCharacterInputs characterInputs = new PlayerCharacterInputs();

			// Build the CharacterInputs struct
			characterInputs.MoveAxisForward = Input.GetAxisRaw(VerticalInput);
			characterInputs.MoveAxisRight = Input.GetAxisRaw(HorizontalInput);
			characterInputs.JumpDown = Input.GetButtonDown("Jump");

			// Apply inputs to character
			Character.SetInputs(ref characterInputs);
		}

		private void HandleCameraInput()
		{

		}
}
}