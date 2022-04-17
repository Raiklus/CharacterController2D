using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class FollowPath : MonoBehaviour {
	public string nodeIcon = "circle.png";
	public string handlerIcon = "triangle.png";
	private string gizmosPath = "2DFollowPath/";

	public List<Transform> points = new List<Transform>();
	public List<NodeBehaviour> nodesBehaviour = new List<NodeBehaviour>();

	[HideInInspector]
	public List<Transform> handlerPoints = new List<Transform>();
	public int segmentsForCurve = 10;

	[HideInInspector]
	public bool _loop = true;
	[HideInInspector]
	public bool _pathReturn = false;

	BezierPath path;

	public bool berzierPath = false;

	public bool loop
	{
		get
		{
			return _loop;
		}
		set
		{
			if(value)
			{
				pathReturn = false;
			}
			_loop = value;
		}
	}
	public bool pathReturn
	{
		get
		{
			return _pathReturn;
		}
		set
		{
			if(value)
			{
				loop = false;
			}
			_pathReturn = value;
		}
	}
	
	private void InitializeBerzierPath()
	{
		path = new BezierPath();
		List<Vector3> vc =  new List<Vector3>();

		int index = 0;
		foreach(Transform tr in points)
		{
			NodePath nodePath = tr.GetComponent<NodePath> ();

			if (nodePath == null) {
				if (index != 0) {
					vc.Add (tr.GetChild (0).position);
				}
				vc.Add (tr.position);
				vc.Add (tr.GetChild (1).position);
			} else {
				if (index != 0) {
					vc.Add (nodePath.handler1 + this.transform.position);
				}
				vc.Add (tr.position);
				vc.Add (nodePath.handler2 + this.transform.position);
			}
			index++;
		}

		if(loop && points.Count > 0)
		{
			NodePath nodePath0 = points[0].GetComponent<NodePath> ();

			if (nodePath0 == null) {
				vc.Add (points [0].GetChild (0).position);
				vc.Add (points [0].position);
			} else {
				vc.Add (nodePath0.handler1 + this.transform.position);
				vc.Add (points [0].position);
			}
		}

		path.SetControlPoints(vc);
	}
	
	private void DrawArrow(Vector3 from , Vector3 to)
	{
		float arrowHeadAngle = 90.0f;

		Vector3 direction = (from - to).normalized;
		if(direction != Vector3.zero)
		{
			Vector3 right = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 + arrowHeadAngle, 0)*new Vector3(0.2f,0.2f,0f);
			Vector3 left = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 - arrowHeadAngle, 0)*new Vector3(0.2f,0.2f,0f);
			
			Gizmos.DrawLine(from, to);
			Vector3 startLine = to;
			Vector3 endLeftLine = (to - left);
			Vector3 endRightLine = (to + right);
			if(!this.berzierPath)
			{
				startLine += (direction/2);
				endLeftLine += (direction/2);
				endRightLine += (direction/2);
			}
			Gizmos.DrawLine(startLine, endLeftLine);
			Gizmos.DrawLine(startLine, endRightLine);
		}
	}

	public NodeBehaviour GetNodeBehaviour(int node)
	{
		foreach(NodeBehaviour n in nodesBehaviour)
		{
			if(n.node == node)
			{
				return n;
			}
		}
		return null;
	}

	public bool IsBerzier(Transform tran)
	{
		if(this.points.Count > 0)
		{
			if(!loop)
			{
				Transform lastPoint = this.points[this.points.Count-1];
				if(lastPoint != null)
				{
					if(lastPoint.GetChild(1) == tran || this.points[0].GetChild(0) == tran)
					{
						return false;
					}
				}
			}
		}
		return this.berzierPath;
	}
	
	public void OnDrawGizmos () {
		Transform pointAnt = null;

		foreach(Transform tran in points)
		{
			if(!berzierPath)
			{
				if(pointAnt != null)
				{
					if(pathReturn)
					{
						Gizmos.color = Color.red;
						DrawArrow(tran.position + new Vector3(0,0.1f, 0), (pointAnt.position + new Vector3(0,0.1f, 0)));
					}

					Gizmos.color = Color.black;
					DrawArrow(pointAnt.position,tran.position);
				}
					
				Gizmos.DrawIcon(tran.position, gizmosPath + nodeIcon, true);
			}

			pointAnt = tran;
		}

		if(!berzierPath)
		{
			if(this.loop)
			{
				if(points.Count > 0)
				{
					if(pointAnt != points[0])
					{
						DrawArrow(points[points.Count - 1].position,points[0].position);
					}
				}
			}
		}
		
		if(berzierPath)
		{
			InitializeBerzierPath();

			if(path != null)
			{
				// Draw Bezier Interpolation
				Gizmos.color = Color.black;
				List<Vector3> pointsV = GetBerzierPoints(this.segmentsForCurve);
				if(pointsV.Count > 1)
				{
					for(int i = 0;i<pointsV.Count;i++)
					{
						//Gizmos.DrawSphere( points[i],0.25f);
						if(i < (pointsV.Count-1))
							DrawArrow(pointsV[i],pointsV[i+1]);
					}
				}

				int index = 0;
				Gizmos.color = Color.yellow;
				foreach(Transform tr in points)
				{
					NodePath nodePath = tr.GetComponent<NodePath> ();
					Vector3 posHandler1 = default(Vector3);
					Vector3 posHandler2 = default(Vector3);

					if (nodePath == null) {
						posHandler1 = tr.GetChild (0).position;
						posHandler2 = tr.GetChild (1).position;
					} else {
						posHandler1 = nodePath.handler1 + this.transform.position;
						posHandler2 = nodePath.handler2 + this.transform.position;
					}

					if (index != 0 || loop) {
						Gizmos.DrawLine (tr.position, posHandler1);
						Gizmos.DrawIcon (posHandler1, gizmosPath + handlerIcon, true);
					}
					if (index != points.Count - 1 || loop) {
						Gizmos.DrawLine (tr.position, posHandler2);
						Gizmos.DrawIcon (posHandler2, gizmosPath + handlerIcon, true);
					}

					Gizmos.DrawIcon(tr.position, gizmosPath + nodeIcon, true);

					index++;
				}
			}
		}
	}

	public List<Vector3> GetBerzierPoints(int _pathSegments)
	{
		if(path == null)
			InitializeBerzierPath();
		
		path.SEGMENTS_PER_CURVE = _pathSegments;
		
		List<Vector3> list = path.GetDrawingPoints(BezierInterpolationMode.Mode1);

		return list;
	}
}
