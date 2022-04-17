using UnityEngine;
using System.Collections;
using UnityEditor;

public static class editorUtils {
	
	public static float GetCameraDist(Vector3 aPt) {
		return Vector3.Distance(SceneView.lastActiveSceneView.camera.transform.position, aPt);
	}
	
	public static float HandleScale  (Vector3 aPos) {
		float dist = SceneView.lastActiveSceneView.camera.orthographic ? SceneView.lastActiveSceneView.camera.orthographicSize / 0.45f : GetCameraDist(aPos);
		return Mathf.Min(0.4f * 1, (dist / 5.0f) * 0.4f * 1);
	}
	
	public static Vector2 GetMousePos(Vector2 aMousePos, float aZOffset) 
	{
		//aMousePos.y = Screen.height - (aMousePos.y + 25);
		Ray   ray   = SceneView.lastActiveSceneView.camera.ScreenPointToRay(new Vector3(aMousePos.x, aMousePos.y, 0));
		Plane plane = new Plane(new Vector3(0,0,-1), aZOffset);
		float dist  = 0;
		Vector3 result = new Vector3(0,0,0);
		
		ray = HandleUtility.GUIPointToWorldRay(aMousePos);
		if (plane.Raycast(ray, out dist)) {
			result = ray.GetPoint(dist);
		}
		return new Vector2(result.x, result.y);
	}
	
	public static bool IsVisible(Vector3 aPos) 
	{
		Transform t = SceneView.lastActiveSceneView.camera.transform;
		if (Vector3.Dot(t.forward, aPos - t.position) > 0)
			return true;
		return false;
	}

	public static void DrawArrow(Vector3 from , Vector3 to)
	{
		float arrowHeadAngle = 90.0f;
		
		Vector3 direction = (from - to).normalized;
		if(direction != Vector3.zero)
		{
			Vector3 right = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 + arrowHeadAngle, 0)*new Vector3(0.2f,0.2f,0f);
			Vector3 left = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 - arrowHeadAngle, 0)*new Vector3(0.2f,0.2f,0f);
			
			Handles.DrawLine(from, to);
			Vector3 startLine = to;
			Vector3 endLeftLine = (to - left);
			Vector3 endRightLine = (to + right);
			startLine += (direction/2);
			endLeftLine += (direction/2);
			endRightLine += (direction/2);
			Handles.DrawLine(startLine, endLeftLine);
			Handles.DrawLine(startLine, endRightLine);
		}
	}

	public static Texture2D GetGizmo(string aFileName) {
		Texture2D tex = AssetDatabase.LoadAssetAtPath("Assets/Gizmos/" + aFileName, typeof(Texture2D)) as Texture2D;
		if (tex == null) {
			tex = EditorGUIUtility.whiteTexture;
			Debug.Log("Couldn't load Gizmo tex " + aFileName);
		}
		return tex;
	}

	public static void    SetScale     (Vector3 aPos, Texture aIcon, ref GUIStyle aStyle, float aScaleOverride = 1) {
		float max      = (Screen.width + Screen.height) / 2;
		float dist     = SceneView.lastActiveSceneView.camera.orthographic ? SceneView.lastActiveSceneView.camera.orthographicSize / 0.5f : editorUtils.GetCameraDist(aPos);
		
		float div = (dist / (max / 160));
		float mul = 0.5f;
		
		aStyle.fixedWidth  = (aIcon.width  / div) * mul;
		aStyle.fixedHeight = (aIcon.height / div) * mul;
	}

	public static Vector3 GetSpawnPos() {
		Plane   plane  = new Plane(new Vector3(0, 0, -1), 0);
		float   dist   = 0;
		Vector3 result = new Vector3(0, 0, 0);
		//Ray     ray    = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition);
		Ray ray = SceneView.lastActiveSceneView.camera.ViewportPointToRay(new Vector3(0.5f, 0.5f, 1.0f));
		if (plane.Raycast(ray, out dist)) {
			result = ray.GetPoint(dist);
		}
		return new Vector3(result.x, result.y, 0);
	}
}
