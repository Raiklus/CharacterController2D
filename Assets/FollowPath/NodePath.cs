using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NodePath : MonoBehaviour {
	public Vector3 handler1;
	public Vector3 handler2;

	[HideInInspector]
	public Vector3 getHandlerPositionByIndex(int index)
	{
		if (index == 0) {
			return handler1;
		} else if (index == 1){
			return handler2;
		}

		return default(Vector3);
	}
	public void setHandlerByIndex(int index, Vector3 position)
	{
		if (index == 0) {
			handler1 = position;
		} else if (index == 1){
			handler2 = position;
		}
	}
}
