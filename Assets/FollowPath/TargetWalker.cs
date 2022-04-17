using System;
using UnityEngine;
using System.Collections.Generic;
using UnityEngine.Events;
using CooldHands;


public class TargetWalker : BaseMoverController
{
		[Serializable]
		public class TargetWalkerFinishEvent : UnityEvent
		{

		}


		public float m_speed = 5.0f;
	
		public FollowPath m_pathController;
	
		private bool m_rotate = false;
		public bool m_running = false;
		public bool m_keepRotation = false;
		public bool m_runningOnlyWhenVisible = true;
		public bool m_usePhysics = false;
		public bool m_useSmoothStep = false;
		private bool m_constantGlobalSpeed = true;
		private bool m_return = false;
		//	private int m_pathSegments = 0;
		public int m_currentNode = 0;
		private bool back = false;
	
		private List<Vector3> m_list = new List<Vector3> ();
		private Vector3 m_lastStartPoint;
		private Quaternion m_lastStartRotation;
		private float m_currentDistance = 1;
		private float m_time = 0;
		private int manualNextNode = 0;

		public delegate void ChangeNode (int node,Transform pointTransform);

		public float startdelay = 0f;
		public ChangeNode OnChangeNode;
		private bool manualMoveNode = false;
		//	private bool tmpRunning = false;
		private int actualRealNode = 0;
		private int tmpRealNode = 0;
		private float waitTime = 0;
		private float startTime = 0;
		private float useTime;
		private Rigidbody2D bodyPhysics;
		private Renderer rendererSprite;
		Vector2 previousPosition;
		public bool SetStartPosition = false;
		public bool m_DeactiveOnFinish = false;
		private bool m_isWaiting = true;

		public TargetWalkerFinishEvent OnFinish;
		public TargetWalkerFinishEvent OnRunning;
		public TargetWalkerFinishEvent OnWait;
		public TargetWalkerFinishEvent OnStop;

		[HideInInspector]
		public Vector2 CurrentVelocity;

		void Awake ()
		{

		}

		public void SetUseSmoothStep(bool use)
		{
				m_useSmoothStep = use;
		}

		public void SetCurrentNode (int node)
		{
				m_currentNode = node;
		}

		void Start ()
		{
				if (SetStartPosition && m_pathController != null && m_pathController.points.Count > 0) {
					//bodyPhysics.position = m_pathController.points [m_currentNode].position;	
					Mover.SetPosition (m_pathController.points [m_currentNode].position);
				}

				if (m_pathController != null) {
						if (!m_pathController.berzierPath) {
								foreach (Transform vec in m_pathController.points) {
										m_list.Add (vec.position);
								}
						} else {
								m_list = m_pathController.GetBerzierPoints (m_pathController.segmentsForCurve);
						}

//			m_pathSegments = m_list.Count;
						m_rotate = m_pathController.loop;
						m_return = m_pathController.pathReturn;

						m_time = 0;
						m_lastStartPoint = this.transform.position;
						m_lastStartRotation = this.transform.rotation;
						m_currentDistance = Vector3.Distance (m_lastStartPoint, m_list [m_currentNode]);

						if (!m_usePhysics) {
								startTime = Time.time;
						} else {
								startTime = Time.time;
						}

						this.waitTime = startdelay;

						NodeBehaviour n = m_pathController.GetNodeBehaviour (m_currentNode);
						if (n != null ) {
								this.m_speed = n.speed;
								this.waitTime = n.waitTime;
						}
				}

				if (m_usePhysics) {
						bodyPhysics = this.GetComponent<Rigidbody2D> ();
				}

				rendererSprite = this.GetComponent<Renderer> ();
		}

		void OnEnable ()
		{
				startTime = Time.time;
		}

		public override void UpdateMovement(ref Vector2 goalPosition, ref Quaternion goalRotation, float deltaTime)
		{
				if (m_usePhysics) {
						if (m_pathController != null) {
								m_rotate = m_pathController.loop;
								m_return = m_pathController.pathReturn;
						}

			MoveTarget (ref goalPosition, ref goalRotation, deltaTime);
				}
		}
		

	void MoveTarget (ref Vector2 goalPosition, ref Quaternion goalRotation, float deltaTime)
		{
				if ((rendererSprite != null && rendererSprite.isVisible && m_runningOnlyWhenVisible) || !m_runningOnlyWhenVisible) {
						float currentTime = 0;
						if (!m_usePhysics) {
								currentTime = Time.time;
						} else {
								currentTime = Time.time;
						}

						if ((currentTime < (startTime + waitTime)) && waitTime != 0) {
								if (OnWait != null && !m_isWaiting) {
										OnWait.Invoke ();	
								}
								m_isWaiting = true;
								return;
						}
			
						if (!m_running)
								return;


						if (OnRunning != null && m_isWaiting) {
								OnRunning.Invoke ();
						}

						m_isWaiting = false;
			
						if (m_list == null || m_list.Count == 0)
								return;
			
						m_time = Mathf.Clamp01 (deltaTime * m_speed / m_currentDistance + m_time);
					

						if (m_time == 1) {
								
								m_time = 0;

							m_lastStartPoint = goalPosition;

				m_lastStartRotation = goalRotation;
				
								if (manualNextNode == m_currentNode && manualMoveNode) {
										manualMoveNode = false;
										this.Stop ();
								}
				
								if (m_return && !back && m_currentNode == m_list.Count - 1) {
										back = true;
								}
				
								if (m_return && back && m_currentNode == 0) {
										back = false;
								}
				
								if (m_rotate && back && m_currentNode == 0) {
										m_currentNode = m_list.Count;
								}
				
								if (m_pathController.berzierPath) {
										if ((m_currentNode % m_pathController.segmentsForCurve) == 0) {
												actualRealNode = (m_currentNode) / m_pathController.segmentsForCurve;
										}
					
										if (m_list.Count - 1 == m_currentNode && m_pathController.loop) {
												actualRealNode = 0;
										}
								} else {
										actualRealNode = m_currentNode;
								}
				
								waitTime = 0;
				
								NodeBehaviour n = m_pathController.GetNodeBehaviour (actualRealNode);
								if (n != null && actualRealNode != tmpRealNode) {
										this.m_speed = n.speed;
										this.waitTime = n.waitTime;
					
										if (!m_usePhysics) {
												startTime = Time.time;
										} else {
												startTime = Time.time;
										}
								}
				
				
								if (actualRealNode != tmpRealNode) {
										//Transform trans = m_pathController.gameObject.transform.Find("point"+actualRealNode);
										Transform trans = m_pathController.points [actualRealNode];
										if (trans != null) {						 
												if (OnChangeNode != null) {	
														OnChangeNode (actualRealNode, trans);
												}
												tmpRealNode = actualRealNode;
						
												if (!m_usePhysics) {
														this.transform.position = trans.position;
														//m_lastStartPoint = trans.position;
												} else {
														MoveCaughtObjects (new Vector2 (trans.position.x, trans.position.y));
														//bodyPhysics.MovePosition (new Vector2 (trans.position.x, trans.position.y));
														//goalPosition = new Vector2 (trans.position.x, trans.position.y);
														//aqui nuevo
														//m_lastStartPoint = bodyPhysics.position;
														//m_lastStartPoint = new Vector2 (trans.position.x, trans.position.y);
												}
										}
								}
				
				
								if (!back) {
										m_currentNode++;
								} else {
										m_currentNode--;
								}
				
								if (m_rotate) {
										m_currentNode = m_currentNode % (m_list.Count);
								} else {
										if ((m_currentNode > m_list.Count - 1 || m_currentNode < 0) && !m_return) {
												m_running = false;
												if (m_DeactiveOnFinish) {
														this.gameObject.SetActive (false);
												}

												if (OnFinish != null) {
														OnFinish.Invoke ();
												}
												return;
										}
								}
								if (m_constantGlobalSpeed)
										m_currentDistance = Vector3.Distance (m_lastStartPoint, m_list [m_currentNode]);
				
						}
			
						if (!m_keepRotation) {
								Vector3 direction = (m_list [m_currentNode] - this.transform.position).normalized;
								float angle = Vector2.Angle (direction, Vector2.up);

								//Vector3 direction = (m_list [m_currentNode] - this.transform.position);
								//var angle = Mathf.Atan2 (direction.y, direction.x) * Mathf.Rad2Deg;

								if (this.transform.position.x < m_list [m_currentNode].x) {
										angle = -1 * angle;
								}
								Quaternion qua = new Quaternion ();
								qua.eulerAngles = new Vector3 (0, 0, angle);
								float timeRotation = m_time;
				
								if (!m_pathController.berzierPath) {
										timeRotation = m_time * 4;
								}
				
								if (!m_usePhysics) {
										this.transform.rotation = Quaternion.Lerp (m_lastStartRotation, qua, timeRotation);
								} else {
									goalRotation = Quaternion.Lerp (m_lastStartRotation, qua, timeRotation); 
										//bodyPhysics.rotation = (Mathf.Lerp(m_lastStartRotation.eulerAngles.z, qua.eulerAngles.z, timeRotation));
								}
								//this.transform.rotation = qua;
						}
						useTime = m_time;
						if (m_useSmoothStep) {
				

								useTime = 1.0f - Mathf.Cos(m_time * Mathf.PI * 0.5f);
						}

						if (!m_usePhysics) {
								
								this.transform.position = Vector3.Lerp (m_lastStartPoint, m_list [m_currentNode], useTime);
						} else {
				Vector2 newPos = Vector2.Lerp (new Vector2 (m_lastStartPoint.x, m_lastStartPoint.y), new Vector2 (m_list [m_currentNode].x, m_list [m_currentNode].y), useTime);
								MoveCaughtObjects (newPos);
								goalPosition = newPos;
								//bodyPhysics.MovePosition (newPos);
						}
				}
		}

		private void MoveCaughtObjects (Vector2 newPos)
		{
				Vector2 direction = newPos - previousPosition;
				previousPosition = newPos;
				CurrentVelocity = (direction / Time.deltaTime) * (1 - m_time);
			
				/*if (platformCatcher != null) {
						Vector2 direction = newPos - previousPosition;
						previousPosition = newPos;
						Vector2 m_Velocity = direction;
						platformCatcher.MoveCaughtObjects (m_Velocity);
				}*/
		}

		public void reset (bool run)
		{
				this.SetCurrentNode (0);
				this.m_running = run;
				this.transform.position = m_pathController.points [m_currentNode].position;
				m_lastStartPoint = this.transform.position;
				m_time = 0;
				m_currentDistance = Vector3.Distance (m_lastStartPoint, m_list [m_currentNode]);
				this.transform.rotation = Quaternion.Euler (new Vector3 (0, 0, 0));
		}

		public void resetNoRotation (bool run)
		{
				this.SetCurrentNode (0);
				this.m_running = run;
				this.transform.position = m_pathController.points [m_currentNode].position;
				m_lastStartPoint = this.transform.position;
				m_time = 0;
				m_currentDistance = Vector3.Distance (m_lastStartPoint, m_list [m_currentNode]);
		}

		public void SetPositionToNode(int node)
		{
				this.SetCurrentNode (node);
				this.transform.position = m_pathController.points [m_currentNode].position;
				m_lastStartPoint = this.transform.position;
				m_time = 0;
				m_currentDistance = Vector3.Distance (m_lastStartPoint, m_list [m_currentNode]);
		}

		public void resetAndSetNextNode (int node)
		{
				m_currentNode = node;
				m_time = 0;
				manualMoveNode = true;
				this.Run ();
		}

		public void setNextNode (int node)
		{
				int tmpNode = node;
				if (m_pathController.berzierPath) {
						tmpNode = m_pathController.segmentsForCurve * node;
				}
		
				if (manualNextNode != tmpNode) {
						if (node > m_currentNode) {
								back = false;
						}

						if (node < m_currentNode) {
								back = true;
						}

						if (m_pathController.berzierPath) {
								manualNextNode = m_pathController.segmentsForCurve * node;
						} else {
								manualNextNode = node;
						}

						manualMoveNode = true;
						m_time = 1;
						this.Run ();
				}
		}

		public int CurrentNode {
				get {
						return actualRealNode;
				}
		}

		public void Run ()
		{
				if (m_usePhysics && bodyPhysics != null && !bodyPhysics.simulated) {
						bodyPhysics.simulated = true;
				}
				m_running = true;
		}

		public void Stop ()
		{
				m_running = false;
				if (OnStop != null) {
						OnStop.Invoke ();
				}
		}
}

