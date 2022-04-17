using UnityEngine;
using System.Collections;
using UnityEditor;

public class editorMenuFollowPath {

	[MenuItem("GameObject/Follow Path/Create Follow Path %#R", false, 0)]
	static void MenuCreateFollowPath() {
		CreateFollowPath();
	}

	static void CreateFollowPath() {
		GameObject obj = new GameObject("New Follow Path");
		obj.AddComponent<FollowPath>();
		obj.transform.position = editorUtils.GetSpawnPos();
		Selection.activeObject = obj;
		EditorGUIUtility.PingObject(obj);
	}
}
