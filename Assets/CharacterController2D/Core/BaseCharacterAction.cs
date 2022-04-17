using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CooldHands
{
	public class CharacterActionResult
	{
		public Vector2 velocity;
		public Quaternion rotation;
	}

	public abstract class BaseCharacterAction : MonoBehaviour {

		public CharacterController2D Controller2D;
		public object[] Parameters;

		/// <summary>
		/// Execute action
		/// </summary>
		public virtual CharacterActionResult Execute(float deltaTime){ return null; }


		/// <summary>
		/// Exit action
		/// </summary>
		public virtual CharacterActionResult ExitExecute(float deltaTime){ return null;}

		/// <summary>
		/// Prepare action to execute
		/// </summary>
		public virtual void RequestAction(params object[] parameters) {
			Parameters = parameters;
		}
	}
}
