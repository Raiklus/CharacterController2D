using UnityEngine;
using System.Collections;
using UnityEditor;
using System;
using System.Reflection;
using System.Collections.Generic;

[CustomEditor(typeof(FollowPath))] 
public class editorFollowPath : Editor {

	public static Vector3 offset    = new Vector3(0, 0, -0.0f);

	Vector2 dragStart;
//	bool drag = false;
	List<int>  selectedPoints = new List<int>();
	List<int>  selectedHandles = new List<int>();

	public override void OnInspectorGUI()
	{
		base.OnInspectorGUI();
		(this.target as FollowPath).loop = GUILayout.Toggle((this.target as FollowPath).loop, "Loop");
		(this.target as FollowPath).pathReturn = GUILayout.Toggle((this.target as FollowPath).pathReturn, "Path Return");

		if (GUILayout.Button("Add"))
		{
			FollowPath path = (this.target as FollowPath);
			Vector3 pos = path.transform.position;
			
			if(path.points.Count > 0)
			{
				pos = path.points[path.points.Count - 1].position;
			}
			addPoint(pos);
		}

		EditorUtility.SetDirty((this.target as FollowPath).gameObject);
	}


	private void OnSceneGUI() {
		if (Event.current.shift) {
			AddHandlePoint();
		}

		DoHandles();
	}

	private void AddHandlePoint()
	{
		FollowPath path = (this.target as FollowPath);

		Vector2  pos = editorUtils.GetMousePos(Event.current.mousePosition, path.transform.position.z) - new Vector2(path.transform.position.x, path.transform.position.y);
		
		Handles.color = new Color(1, 1, 1, 1);
		
		Vector3 handlePos = new Vector3(pos.x, pos.y, 0) + path.transform.position + offset;
		
		if(path.points.Count > 0)
		{
			editorUtils.DrawArrow(path.points[path.points.Count-1].position, handlePos);
			
			if(path.loop)
			{
				editorUtils.DrawArrow(handlePos, path.points[0].position);
			}
		}
		
		//GUIStyle     iconStyle = new GUIStyle();
		Handles.color = new Color(1, 1, 1, 0);
		if (editorUtils.IsVisible(handlePos)) {
			if (Handles.Button(handlePos, SceneView.lastActiveSceneView.camera.transform.rotation, editorUtils.HandleScale(handlePos), editorUtils.HandleScale(handlePos), Handles.CircleHandleCap))
			{
				addPoint(handlePos);
				Selection.activeGameObject = path.gameObject;
			}
		}
	}

	private void DoHandles()
	{
		List<Transform> delete = new List<Transform>();
		FollowPath path = (this.target as FollowPath);

		if(path.points.Count == 0)
		{
			Handles.color = Color.white;
			Vector3 p = path.transform.position;
			GUIStyle s = new GUIStyle();
			Handles.Label(p, "Press SHIFT to create point, CONTROL to delete", s);
		}

		Handles.color = new Color(1, 1, 1, 0);
		//GUIStyle     iconStyle = new GUIStyle();

		for (int i = 0; i < path.points.Count; i++)
		{
			Vector3 pos        = path.points[i].position;
			Vector3 posOff     =  pos + new Vector3(-0.37f, 0.37f, 0);

			if(!Event.current.control)
			{
				bool isSelected = selectedPoints.Contains(i);

				Vector3 snap   = Event.current.control ? new Vector3(EditorPrefs.GetFloat("MoveSnapX"), EditorPrefs.GetFloat("MoveSnapY"), EditorPrefs.GetFloat("MoveSnapZ")) : Vector3.zero;
				EditorGUI.BeginChangeCheck();

				Vector3 result = Handles.FreeMoveHandle(
					pos,
					SceneView.lastActiveSceneView.camera.transform.rotation,
					editorUtils.HandleScale(posOff),
					snap, 
					Handles.CircleHandleCap);

				if (EditorGUI.EndChangeCheck ()) {
					if (isSelected == false) {
						selectedPoints.Clear();
						selectedPoints.Add(i);
						isSelected = true;
					}

					for (int s = 0; s < selectedPoints.Count; s++) {
						Undo.RegisterFullObjectHierarchyUndo(path.points[selectedPoints[s]].gameObject, "Node move");
	

						if (!path.points [selectedPoints [s]].position.Equals (result)) {
							Vector3 fixMove = path.points [selectedPoints [s]].position - result;
							path.points [selectedPoints [s]].position = result;

							NodePath nodePath = path.points [selectedPoints [s]].GetComponent<NodePath> ();
							if (nodePath != null) {
								Undo.RecordObject (nodePath, "Node move");						
								nodePath.handler1 -= fixMove;
								nodePath.handler2 -= fixMove;
							}
						}

					}
				}

				if(path.berzierPath)
				{
					for(int z = 0; z <  2; z++)
					{
						bool isHandleSelected = selectedHandles.Contains(z);
						
						Vector3 snap2 = Event.current.control ? new Vector3(EditorPrefs.GetFloat("MoveSnapX"), EditorPrefs.GetFloat("MoveSnapY"), EditorPrefs.GetFloat("MoveSnapZ")) : Vector3.zero;

						EditorGUI.BeginChangeCheck();

						Vector3 resultHandle = default(Vector3);

						NodePath nodePath = path.points[i].GetComponent<NodePath> ();

						if (nodePath == null) {
							resultHandle = Handles.FreeMoveHandle (
								path.points [i].GetChild (z).position,
								SceneView.lastActiveSceneView.camera.transform.rotation,
								editorUtils.HandleScale (path.points [i].GetChild (z).position),
								snap2, 
								Handles.RectangleHandleCap);
						} else {
							resultHandle = Handles.FreeMoveHandle (
								nodePath.getHandlerPositionByIndex(z)+path.transform.position,
								SceneView.lastActiveSceneView.camera.transform.rotation,
								editorUtils.HandleScale (nodePath.getHandlerPositionByIndex(z)+path.transform.position),
								snap2, 
								Handles.RectangleHandleCap);
						}

						if (EditorGUI.EndChangeCheck ()) {

							if (nodePath == null) {
								UnityEngine.Object[] undoObjects = new UnityEngine.Object[2];
								//undoObjects [0] = path;
								undoObjects [0] = path.points [i].GetChild (0);
								undoObjects [1] = path.points [i].GetChild (1);
								Undo.RecordObjects (undoObjects, "Handles move");
							} else {
								//Undo.RecordObject(path.points [i], "Handles move");
								Undo.RecordObject (nodePath, "Handles move");
							}

							if (isHandleSelected == false) {
								selectedHandles.Clear ();
								selectedHandles.Add (z);
								isHandleSelected = true;
							}
							
							for (int j = 0; j < selectedHandles.Count; j++) {
								
								if (nodePath == null) {
									path.points [i].GetChild (selectedHandles [j]).position = resultHandle;
									if (selectedHandles [j] == 0) {
										path.points [i].GetChild (1).localPosition = new Vector3 (path.points [i].GetChild (selectedHandles [j]).localPosition.x * -1, path.points [i].GetChild (selectedHandles [j]).localPosition.y * -1, path.points [i].GetChild (selectedHandles [j]).localPosition.z * -1);
									} else {
										path.points [i].GetChild (0).localPosition = new Vector3 (path.points [i].GetChild (selectedHandles [j]).localPosition.x * -1, path.points [i].GetChild (selectedHandles [j]).localPosition.y * -1, path.points [i].GetChild (selectedHandles [j]).localPosition.z * -1);
									}
									
								} else {
									Vector3 resulte =  (nodePath.getHandlerPositionByIndex(selectedHandles [j]) - (resultHandle - path.transform.position));
									nodePath.setHandlerByIndex (selectedHandles [j], resultHandle - path.transform.position);
									if (selectedHandles [j] == 0) {
										nodePath.handler2 = nodePath.handler2 + resulte;
									} else {
										nodePath.handler1 = nodePath.handler1 + resulte;
									}
									
								}
							}
						}
					}
				}
			}
			else
			{
				Handles.color = Color.red;

				if (editorUtils.IsVisible(pos)) {
					if (Handles.Button(pos, SceneView.lastActiveSceneView.camera.transform.rotation, editorUtils.HandleScale(pos), editorUtils.HandleScale(pos), Handles.CircleHandleCap))
					{
						delete.Add(path.points[i]);
					}
				}
			}
		}

		foreach(Transform tran in delete)
		{
			//Undo.DestroyObjectImmediate(tran.gameObject);
			//Undo.RegisterFullObjectHierarchyUndo(path, "Delete node");
			//Undo.DestroyObjectImmediate(tran.gameObject);
			UnityEngine.Object[] undoObjects = new UnityEngine.Object[2];
			//undoObjects [0] = path;
			undoObjects [0] = tran.gameObject;
			undoObjects [1] = path;

			Undo.RegisterCompleteObjectUndo(undoObjects, "Delete node");
			Undo.DestroyObjectImmediate(tran.gameObject);

			path.points.Remove(tran);
		}
	}

	private Vector3 RotateAroundPoint(Vector3 point , Vector3 pivot, Quaternion angle) {
		return angle * ( point - pivot) + pivot;
	}
		

	// Update is called once per frame
	private void addPoint(Vector3 pos)
	{
		FollowPath path = (this.target as FollowPath);
		//Undo.RecordObject(path, "New node");
		Undo.RegisterFullObjectHierarchyUndo (path, "New node");

		GameObject point = new GameObject("point");
		point.transform.parent = path.transform;
		NodePath nodePath = point.AddComponent <NodePath>();

		point.transform.position = pos;

		Vector3 lastPos = default(Vector3);
		if (path.points.Count > 0) {
			lastPos = path.points [path.points.Count - 1].position;
		}

		Vector3 targetDir = lastPos - point.transform.position;
		Quaternion rot = Quaternion.LookRotation (targetDir);

		Vector3 point1 = RotateAroundPoint(point.transform.position + new Vector3(0,0,3),point.transform.position, rot);
		Vector3 point2 = RotateAroundPoint(point.transform.position + new Vector3(0,0,-3),point.transform.position, rot);

		nodePath.handler1 = RotateAroundPoint(point1,point.transform.position, Quaternion.Euler(0,0,70)) - path.transform.position;
		nodePath.handler2 = RotateAroundPoint(point2,point.transform.position, Quaternion.Euler(0,0,70))- path.transform.position;

		path.points.Add(point.transform);
		Undo.RegisterCreatedObjectUndo(point, "New node");

	}
}
