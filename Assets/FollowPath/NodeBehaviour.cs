using UnityEngine;
using System.Collections;

public class NodeBehaviour : MonoBehaviour {
	public FollowPath path;
	public int node;
	public float waitTime;
	public float speed;

	void Start()
	{
		if(path != null)
		{
			path.nodesBehaviour.Add(this);
		}
	}
}

