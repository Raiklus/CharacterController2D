﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CooldHands
{
    public abstract class BaseMoverController : MonoBehaviour
    {
        /// <summary>
        /// The PhysicsMover that will be assigned to this MoverController via the inspector
        /// </summary>
        public PhysicsMover Mover { get; private set; }

        /// <summary>
        /// This is called by the PhysicsMover in its Awake to setup references
        /// </summary>
        public void SetupMover(PhysicsMover mover)
        {
            Mover = mover;
        }

        /// <summary>
        /// Asks for the new position and rotation that the mover should have on this update
        /// </summary>
        public abstract void UpdateMovement(ref Vector2 goalPosition, ref Quaternion goalRotation, float deltaTime);
    }
}