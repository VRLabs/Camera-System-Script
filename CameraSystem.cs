using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Numerics;
using Rug.Osc;
using VRC.OSCQuery;
using System.Windows.Forms;

public class CameraSystem
{
	public const string xRotReceiveParamPath			 = "CameraSystem/Send/AngleX_Angle";	// Float | absolute x rotation [0, 1]
	public const string xRotReceiveBoolParamPath		 = "CameraSystem/Send/PosNegX_Angle";	// Float | sign of x rotation
	public const string yRotReceiveParamPath			 = "CameraSystem/Send/AngleY_Angle";	// Float | absolute y rotation [0, 1]
	public const string yRotReceiveBoolParamPath		 = "CameraSystem/Send/PosNegY_Angle";	// Float | sign of y rotation
	public const string zRotReceiveParamPath			 = "CameraSystem/Send/AngleZ_Angle";	// Float | absolute z rotation [0, 1]
	public const string zRotReceiveBoolParamPath		 = "CameraSystem/Send/PosNegZ_Angle";	// Float | sign of z rotation
	public const string xPosReceiveParamPath			 = "CameraSystem/Send/PositionX";		// Float | x position [0, 1]
	public const string yPosReceiveParamPath			 = "CameraSystem/Send/PositionY";		// Float | y rotation [0, 1]
	public const string zPosReceiveParamPath			 = "CameraSystem/Send/PositionZ";		// Float | z rotation [0, 1]
	public const string pointIndexParamPath				 = "CameraSystem/PointIndex";			// Int	 | current point to edit
	public const string setPointParamPath				 = "CameraSystem/SetPoint";				// Bool  | set current selected point to current position/rotation location
	public const string totalTimeParamPath				 = "CameraSystem/TotalTimeLength";		// Float | time we want the full track to take (0 = 0 sec, 1 = 10 sec)
	public const string resetAllPointsParamPath			 = "CameraSystem/ResetAllPoints";		// Bool  | resets every point
	public const string playTrackParamPath			 	 = "CameraSystem/PlayTrack";			// Bool  | starts playing the current track 
	public const string isPlayingParamPath				 = "CameraSystem/IsPlaying";			// Bool  | we set this when playing
	public const string isTimePerPointParamPath			 = "CameraSystem/IsTimePerPoint";  		// Bool  | Whether or not we use total time or time per point
	public const string waitTimeParamPath				 = "CameraSystem/WaitTime";  			// Float | how long to wait after done (0 = 0 sec, 1 = 10 sec)
	public const string useBSplineParamPath				 = "CameraSystem/UseBSpline";			// Bool  | whether or not to use a b spline
	public const string loopPath						 = "CameraSystem/Loop";					// Bool  | whether or not we loop the system
	public const string closedLoopPath					 = "CameraSystem/ClosedLoop";			// Bool  | whether we should close the loop, seamlessly looping back to the first point
	public const string circleModePath					 = "CameraSystem/CircleMode";			// Bool  | whether we should rotate around a point or follow a path
	public const string errorCodeParamPath				 = "CameraSystem/ErrorCode";			// Int   | we set this when we error
	public const string connectedPath					 = "CameraSystem/Connected";			// Bool  | we set this to true to show we are connected
	public const string xPosSendParamPath				 = "CameraSystem/Receive/XPosition";	// Float | Current x location of the camera [0, 1]
	public const string yPosSendParamPath				 = "CameraSystem/Receive/YPosition";	// Float | Current y location of the camera [0, 1]
	public const string zPosSendParamPath				 = "CameraSystem/Receive/ZPosition";	// Float | Current z location of the camera [0, 1] 
	public const string xRotSendParamPath				 = "CameraSystem/Receive/XRotation";	// Float | Current x euler rotation of the camera [0, 1]
	public const string yRotSendParamPath				 = "CameraSystem/Receive/YRotation";	// Float | Current y euler rotation of the camera [0, 1]
	public const string zRotSendParamPath				 = "CameraSystem/Receive/ZRotation";	// Float | Current z euler rotation of the camera [0, 1]


	/*
	 * Error codes:
	 * 1: Total time is 0
	 * 2: Not enough points: 2 points or more needed
	 */

	public static bool ShouldStop = false;
		
	public static async Task Main()
	{
		NotifyIcon notifyIcon = new NotifyIcon();
		notifyIcon.Icon = new Icon("image.ico");
		notifyIcon.Text = "CameraSystem";
		bool shown = false;
		notifyIcon.Click += (sender, args) =>
		{
			if (shown)
			{
				notifyIcon.ContextMenuStrip.Hide();
			}
			else
			{
				notifyIcon.ContextMenuStrip.Show(Cursor.Position);
			}

			shown = !shown;
		};
		
		ContextMenuStrip contextMenu = new ContextMenuStrip();
		contextMenu.Items.Add("Exit");
		contextMenu.Items[0].Click += (sender, args) =>
		{
			ShouldStop = true;
			Application.Exit();
		};
		
		notifyIcon.ContextMenuStrip = contextMenu;
		
		notifyIcon.Visible = true;
		
		new Thread(RunOSCQuery).Start();
		
		Application.Run();
	}

	public static async void RunOSCQuery()
	{
		while (!ShouldStop)
		{
			try
			{
				var tcpPort = Extensions.GetAvailableTcpPort();
				var udpPort = Extensions.GetAvailableUdpPort();
				var oscQuery = new OSCQueryServiceBuilder()
					.WithTcpPort(tcpPort)
					.WithUdpPort(udpPort)
					.WithServiceName("CameraSystem")
					.AdvertiseOSC()
					.AdvertiseOSCQuery()
					.StartHttpServer()
					.Build();
				
				while (!ShouldStop)
				{
					await Task.Delay(100);
					connected = false;
					List<OSCQueryServiceProfile> oscQueryServiceProfiles = oscQuery.GetOSCQueryServices().ToList();
					OSCQueryServiceProfile? oscQueryServiceProfile =
						oscQueryServiceProfiles.FirstOrDefault(x => x.name.Contains("VRChat-Client"));
					if (oscQueryServiceProfile != null)
					{
						await RunVRChat(oscQueryServiceProfile);
					}
				}
			}
			catch (Exception e)
			{
				Console.WriteLine(e.ToString());
			}
		}
	}
	
	public class Point
	{
		public Vector3 position;
		public Quaternion rotation;

		public Point(Vector3 position, Quaternion rotation)
		{
			this.position = position;
			this.rotation = rotation;
		}

		public float GetDistance(Point other)
		{
			return (other.position - position).Length();
		}
	}

	public static float GetFloat(OSCQueryNode node)
	{
		return ((float)(double)node.Value[0]);
	}
	
	public static int GetInt(OSCQueryNode node)
	{
		return ((int)(long)node.Value[0]);
	}
	
	public static bool GetBool(OSCQueryNode node)
	{
		return ((bool)node.Value[0]);
	}

	public static float GetDistance(float distance)
	{
		return distance;
	}

	public static float GetRotation(float absoluteAngle, float sideChecker)
	{
		if (sideChecker < 0.5f)
		{
			return absoluteAngle*180f;
		}
		else
		{
			return (-absoluteAngle)*180f;
		}
	}
	
	static 	bool connected = false;

	public static async Task<OSCQueryRootNode?> GetTree(IPAddress ip, int port)
	{
		try
		{
			return await Extensions.GetOSCTree(ip, port);
		}
		catch (Exception e)
		{
			Console.Write(e.StackTrace);
			return null!;
		}
	}

	public static async Task<Dictionary<String, OSCQueryNode>?> GetParameterNodes(IPAddress ip, int port)
	{
		var rootNode = await GetTree(ip, port);
		if (rootNode == null || rootNode.Contents == null) return null;
		if (!rootNode.Contents.TryGetValue("avatar", out var avatarNode)) return null;
		if (!avatarNode.Contents.TryGetValue("parameters", out var parametersNode)) return null;

		Dictionary<String, OSCQueryNode> oscQueryNodes = new Dictionary<string, OSCQueryNode>();

		Stack<OSCQueryNode> todo = new Stack<OSCQueryNode>();
		todo.Push(parametersNode);

		while (todo.Count != 0)
		{
			OSCQueryNode oscQueryNode = todo.Pop();
			if (oscQueryNode != parametersNode)
			{
				oscQueryNodes.Add(oscQueryNode.FullPath.Remove(0, "/avatar/parameters/".Length), oscQueryNode);
			}
			
			if (oscQueryNode.Contents == null) continue;
			
			foreach (var (key, value) in oscQueryNode.Contents)
			{
				todo.Push(value);
			}
		}
		
		return oscQueryNodes;
	}
	
	
	public static async Task RunVRChat(OSCQueryServiceProfile vrchatOsc)
	{
		using (Socket socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp))
		{
			socket.Connect(vrchatOsc.address, 9000);
			Point[] points = new Point[256];
			bool isPointBeingSet = false;
			bool isPlayingBeingSet = false;
			bool isPlaying = false;
			
			int frameIndex = 0;
			int playingFrameIndex = 0;
			int endPlayingFrameIndex = 0;
			
			List<Point> resampled = null;
			
			while (!ShouldStop)
			{
				Stopwatch frameStart = Stopwatch.StartNew();
				Dictionary<string, OSCQueryNode>? oscQueryNodes = await GetParameterNodes(vrchatOsc.address, vrchatOsc.port);
				
				if (oscQueryNodes == null)
				{
					continue;
				}
				
				if (!oscQueryNodes.TryGetValue(xRotReceiveParamPath, out var xRotReceiveParamNode)
					|| !oscQueryNodes.TryGetValue(xRotReceiveBoolParamPath, out var xRotReceiveBoolParamNode)
					|| !oscQueryNodes.TryGetValue(yRotReceiveParamPath, out var yRotReceiveParamNode)
					|| !oscQueryNodes.TryGetValue(yRotReceiveBoolParamPath, out var yRotReceiveBoolParamNode)
					|| !oscQueryNodes.TryGetValue(zRotReceiveParamPath, out var zRotReceiveParamNode)
					|| !oscQueryNodes.TryGetValue(zRotReceiveBoolParamPath, out var zRotReceiveBoolParamNode)
					|| !oscQueryNodes.TryGetValue(xPosReceiveParamPath, out var xPosReceiveParamNode)
					|| !oscQueryNodes.TryGetValue(yPosReceiveParamPath, out var yPosReceiveParamNode)
					|| !oscQueryNodes.TryGetValue(zPosReceiveParamPath, out var zPosReceiveParamNode)
					|| !oscQueryNodes.TryGetValue(pointIndexParamPath, out var pointIndexParamNode)
					|| !oscQueryNodes.TryGetValue(setPointParamPath, out var setPointParamNode)
					|| !oscQueryNodes.TryGetValue(totalTimeParamPath, out var totalTimeParamNode)
					|| !oscQueryNodes.TryGetValue(resetAllPointsParamPath, out var resetAllPointsParamNode)
					|| !oscQueryNodes.TryGetValue(playTrackParamPath, out var playTrackParamNode)
					|| !oscQueryNodes.TryGetValue(isPlayingParamPath, out var isPlayingParamNode)
					|| !oscQueryNodes.TryGetValue(isTimePerPointParamPath, out var isTimePerPointParamNode)
					|| !oscQueryNodes.TryGetValue(waitTimeParamPath, out var waitTimeParamNode)
					|| !oscQueryNodes.TryGetValue(useBSplineParamPath, out var useBSplineParamNode)
					|| !oscQueryNodes.TryGetValue(loopPath, out var loopParamNode)
					|| !oscQueryNodes.TryGetValue(closedLoopPath, out var closedLoopParamNode)
					|| !oscQueryNodes.TryGetValue(circleModePath, out var circleModeParamNode)
					|| !oscQueryNodes.TryGetValue(errorCodeParamPath, out var errorCodeParamNode)
					|| !oscQueryNodes.TryGetValue(connectedPath, out var connectedParamNode)
					|| !oscQueryNodes.TryGetValue(xPosSendParamPath, out var xPosSendParamNode)
					|| !oscQueryNodes.TryGetValue(yPosSendParamPath, out var yPosSendParamNode)
					|| !oscQueryNodes.TryGetValue(zPosSendParamPath, out var zPosSendParamNode)
					|| !oscQueryNodes.TryGetValue(xRotSendParamPath, out var xRotSendParamNode)
					|| !oscQueryNodes.TryGetValue(yRotSendParamPath, out var yRotSendParamNode)
					|| !oscQueryNodes.TryGetValue(zRotSendParamPath, out var zRotSendParamNode))
				{
					continue;
				}


				if (!connected)
				{
					connected = true;
					Console.WriteLine("Connected to VRChat");
				}

				#region Inputs
				// First frames logic
				if (frameIndex == 0)
				{
					socket.Send(new OscMessage(resetAllPointsParamNode.FullPath, new object[] { true }).ToByteArray());
					socket.Send(new OscMessage(totalTimeParamNode.FullPath, new object[] { 0.0f }).ToByteArray());
				}
				
				if (frameIndex == 5)
				{
					socket.Send(new OscMessage(resetAllPointsParamNode.FullPath, new object[] { false }).ToByteArray());
				}
				
				// Handle playing
				if (!isPlayingBeingSet && GetBool(playTrackParamNode))
				{
					isPlayingBeingSet = true;
					Console.WriteLine($"{(isPlaying ? "Stopping" : "Starting")} Playing");
					isPlaying = !isPlaying;
					if (isPlaying) playingFrameIndex = 0;
				}

				if (!GetBool(playTrackParamNode) && isPlayingBeingSet)
				{
					isPlayingBeingSet = false;
				}

				// Handle Setting Points
				if (isPointBeingSet && !GetBool(setPointParamNode))
				{
					isPointBeingSet = false;
					Vector3 position = new Vector3(
						GetDistance(GetFloat(xPosReceiveParamNode)),
						GetDistance(GetFloat(yPosReceiveParamNode)),
						GetDistance(GetFloat(zPosReceiveParamNode)));

					Quaternion rotation = ToQ(new Vector3(GetRotation(GetFloat(xRotReceiveParamNode), GetFloat(xRotReceiveBoolParamNode)),
						GetRotation(GetFloat(yRotReceiveParamNode), GetFloat(yRotReceiveBoolParamNode)),
						GetRotation(GetFloat(zRotReceiveParamNode), GetFloat(zRotReceiveBoolParamNode))));
					points[GetInt(pointIndexParamNode)] = new Point(position, rotation);
					Console.WriteLine($"Added Point {GetInt(pointIndexParamNode)} to {position}, {rotation}");
				}
				
				if (!isPointBeingSet && GetBool(setPointParamNode))
				{
					isPointBeingSet = true;
				}

				// Handle Reset
				if (GetBool(resetAllPointsParamNode))
				{
					points = new Point[256];
					Console.WriteLine("Reset All Points");
				}
				
				#endregion

				#region Movement logic

				if (isPlaying)
				{
					float length = GetFloat(totalTimeParamNode) * 10;

					if (playingFrameIndex == 0)
					{
						// Error Handling
						if (GetFloat(totalTimeParamNode) == 0)
						{
							socket.Send(new OscMessage(errorCodeParamNode.FullPath, new object[] { 1 }).ToByteArray());
							Console.WriteLine($"Sending error 1: Time 0");
							isPlaying = false;
							continue;
						}
						
						Point[] filteredPoints = points.Where(x => x != null).ToArray();
					
						if (filteredPoints.Length < 2)
						{
							socket.Send(new OscMessage(errorCodeParamNode.FullPath, new object[] { 2 }).ToByteArray());
							Console.WriteLine($"Sending error 2: Not enough points");
							isPlaying = false;
							continue;
						}

						
						
						Console.WriteLine($"Starting playing with {filteredPoints.Length} points for duration {length}");

						Point[] finalPoints = GetFinalPointsList(filteredPoints, GetBool(closedLoopParamNode));
						SplineType type = GetBool(useBSplineParamNode) ? SplineType.B : SplineType.CatmulRom;
					
						if (GetBool(circleModeParamNode))
						{
							resampled = SampleCircle(filteredPoints[0], filteredPoints[1], (int)MathF.Floor(length * 100)).ToList();
						}
						else
						{
							if (!GetBool(isTimePerPointParamNode))
							{
								Point[] samples = SampleSpline(finalPoints, type);
								resampled = ResampleCurve(samples, ((int)MathF.Floor(length / 0.01f)));
							}
							else
							{
								resampled = SampleSpline(finalPoints, type, (int)MathF.Floor(length * 100)).ToList();
							}
						}

						endPlayingFrameIndex = resampled.Count + (int)(GetFloat(waitTimeParamNode) * 1000);
					}

					if (playingFrameIndex < resampled.Count)
					{
						Point point = resampled[playingFrameIndex];
						socket.Send(new OscMessage(xPosSendParamNode.FullPath, new object[] { point.position.X }).ToByteArray());
						socket.Send(new OscMessage(yPosSendParamNode.FullPath, new object[] { point.position.Y }).ToByteArray());
						socket.Send(new OscMessage(zPosSendParamNode.FullPath, new object[] { point.position.Z }).ToByteArray());
						socket.Send(new OscMessage(xRotSendParamNode.FullPath, new object[] { FromQ2(point.rotation).X/360f }).ToByteArray());
						socket.Send(new OscMessage(yRotSendParamNode.FullPath, new object[] { FromQ2(point.rotation).Y/360f }).ToByteArray());
						socket.Send(new OscMessage(zRotSendParamNode.FullPath, new object[] { FromQ2(point.rotation).Z/360f }).ToByteArray());
					}

					if (playingFrameIndex == endPlayingFrameIndex)
					{
						if (GetBool(loopParamNode)) playingFrameIndex = -1;
						else isPlaying = false;
					}
					
					playingFrameIndex++;
				}
				

				#endregion
				
				socket.Send(new OscMessage(isPlayingParamNode.FullPath, new object[] { isPlaying }).ToByteArray());
				socket.Send(new OscMessage(connectedParamNode.FullPath, new object[] { true }).ToByteArray());
				frameIndex++;

				while(frameStart.Elapsed.Ticks / (float)TimeSpan.TicksPerMillisecond < 10){}
			}
		}
	}
	
	public static Point[] GetFinalPointsList(Point[] inputPoints, bool closedLoop)
	{
		Point[] points;
		if (closedLoop)
		{
			points = new[] { inputPoints[^1] }.Concat(inputPoints).Append(inputPoints[0]).Append(inputPoints[1]).ToArray();
		}
		else
		{
			points = new[] { inputPoints[1] }.Concat(inputPoints).Append(inputPoints[^2]).ToArray();
		}

		return points;
	}


	public static Point[] SampleCircle(Point center, Point outside, int pointCount)
	{
		float radius = ((center.position - outside.position) with { Y = 0 }).Length();
		Point[] result = new Point[pointCount];

		for (int i = 0; i < pointCount; i++)
		{
			// Calculate the angle for the current point

			Vector3 offset = outside.position - center.position;
			float offsetAngle = MathF.Atan2(offset.Z, offset.X);
			float angle = (float)(2 * Math.PI * i / pointCount + offsetAngle);

			// Calculate the position of the current point
			Vector3 position = new Vector3(
				center.position.X + radius * (float)Math.Cos(angle),
				outside.position.Y,
				center.position.Z + radius * (float)Math.Sin(angle)
			);
			
			// Quaternion additionalRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitY, -angle);

			Vector3 rotationVec = FromQ2(outside.rotation);
			Quaternion rotation = ToQ(rotationVec with { Y = (rotationVec.Y - i / (float)pointCount * 360) });
			result[i] = new Point(position, rotation);
		}

		return result;
	}
	
	//Display a spline between 2 points derived with the Catmull-Rom spline algorithm
	public static Point[] SampleSpline(Point[] pointsList, SplineType type, int pointsPerSpline = 100)
	{
		// if (type == SplineType.PiecewiseHermite)
		// {
		// 	return PiecewiseHermiteInterpolation(pointsList, pointsPerSpline);
		// }
		List<Point> resultPoints = new List<Point>();
		for (int pos = 1; pos < pointsList.Length - 2; pos++)
		{
			//The 4 points we need to form a spline between p1 and p2
			Point p0 = pointsList[pos - 1];
			Point p1 = pointsList[pos];
			Point p2 = pointsList[pos + 1];
			Point p3 = pointsList[pos + 2];

			int loops = pointsPerSpline;

			for (int i = 1; i <= loops; i++)
			{
				//Which t position are we at?
				float t = i / ((float)pointsPerSpline);

				//Find the coordinate between the end points with a Catmull-Rom spline
				Point newPos = GetSplinePosition(t, p0, p1, p2, p3, type);
				resultPoints.Add(newPos);
			}
		}

		return resultPoints.ToArray();
	}

	public enum SplineType
	{
		CatmulRom,
		B,
	}

	//Returns a position between 4 Vector3 with Catmull-Rom spline algorithm
	//http://www.iquilezles.org/www/articles/minispline/minispline.htm
	static Point GetSplinePosition(float t, Point p0, Point p1, Point p2, Point p3, SplineType type)
	{

		Vector3 pos = Vector3.Zero;
		
		Quaternion rot = Quaternion.Identity;
		
		Vector3 p0Euler = FromQ2(p0.rotation);
		Vector3 p1Euler = FromQ2(p1.rotation);
		Vector3 p2Euler = FromQ2(p2.rotation);
		Vector3 p3Euler = FromQ2(p3.rotation);

		p1Euler = CorrectAngles(p0Euler, p1Euler);
		p2Euler = CorrectAngles(p1Euler, p2Euler);
		p3Euler = CorrectAngles(p2Euler, p3Euler);
		
		switch (type)
		{
			case SplineType.CatmulRom:
				pos = CatmulRomSplineInterpolation(p0.position, p1.position, p2.position, p3.position, t);
				rot = ToQ(CatmulRomSplineInterpolation(p0Euler, p1Euler, p2Euler, p3Euler, t));
				break;
			case SplineType.B:
				pos = BSplineInterpolation(p0.position, p1.position, p2.position, p3.position, t);
				rot = ToQ(BSplineInterpolation(p0Euler, p1Euler, p2Euler, p3Euler, t));
				break;
		}

		// rot = Squad.SplineSegment(p0.rotation, p1.rotation, p2.rotation, p3.rotation, t);
		return new Point(pos, rot);
	}

	private static Vector3 BSplineInterpolation(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
	{
		Vector3 a = p0 + 4 * p1 + p2;
		Vector3 b = -3 * p0 + 3 * p2;
		Vector3 c = 3 * p0 + -6 * p1 + 3 * p2;
		Vector3 d = -1 * p0 + 3 * p1 - 3 * p2 + p3;

		//The cubic polynomial: a + b * t + c * t^2 + d * t^3
		return 1f/6f * (a + (b * t) + (c * t * t) + (d * t * t * t));
	}

	public static Vector3 CorrectAngles(Vector3 prev, Vector3 target)
	{
		float diffX = target.X - prev.X;
		float diffY = target.Y - prev.Y;
		float diffZ = target.Z - prev.Z;
		if (diffX > 180)
			diffX -= 360;
		if (diffY > 180)
			diffY -= 360;
		if (diffZ > 180)
			diffZ -= 360;

		if (diffX < -180)
			diffX = 360 + diffX;
		if (diffY < -180)
			diffY = 360 + diffY;
		if (diffZ < -180)
			diffZ = 360 + diffZ;
		return new Vector3(prev.X + diffX, prev.Y + diffY, prev.Z + diffZ);
	}
	
	public static Vector3 CatmulRomSplineInterpolation(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
	{
		//The coefficients of the cubic polynomial (except the 0.5f * which I added later for performance)
		Vector3 a = 2f * p1;
		Vector3 b = p2 - p0;
		Vector3 c = 2f * p0 - 5f * p1 + 4f * p2 - p3;
		Vector3 d = -p0 + 3f * p1 - 3f * p2 + p3;

		//The cubic polynomial: a + b * t + c * t^2 + d * t^3
		return 0.5f * (a + (b * t) + (c * t * t) + (d * t * t * t));
	}

	// public static Point[] PiecewiseHermiteInterpolation(Point[] points, int samplesPerPoint)
	// {
	// 	Vector3[] positions = points.Select(x => x.position).ToArray();
	//
	// 	Vector3[] eulers = new Vector3[points.Length];
	//
	// 	eulers[0] = FromQ2(points[0].rotation);
	// 	for (var i = 1; i < points.Length; i++)
	// 	{
	// 		eulers[i] = CorrectAngles(eulers[i-1], FromQ2(points[i].rotation));
	// 	}
	//
	// 	string positionString = positions.Select(x => $"{x.X}${x.Y}${x.Z}").Aggregate((x, y) => x + "|" + y).Replace(",", ".");
	// 	string eulerString = eulers.Select(x => $"{x.X}${x.Y}${x.Z}").Aggregate((x, y) => x + "|" + y).Replace(",", ".");
	//
	// 	
	// 	ProcessStartInfo start = new ProcessStartInfo();
	// 	start.FileName = "C:\\Program Files (x86)\\Steam\\steamapps\\common\\Blender\\3.6\\python\\bin\\python.exe";
	// 	start.Arguments = $"C:\\Users\\jelle\\Desktop\\School\\Projects\\GeKut\\CameraSystem\\PythonShit\\main.py {positionString} {eulerString} {samplesPerPoint}";
	// 	start.UseShellExecute = false;
	// 	start.RedirectStandardOutput = true;
	// 	string result = "";
	// 	using(Process process = Process.Start(start))
	// 	{
	// 		using(StreamReader reader = process.StandardOutput)
	// 		{
	// 			result = reader.ReadToEnd().Replace(".", ",");
	// 		}
	// 	}
	//
	// 	string[] outputs = result.Split("^");
	// 	Vector3[] newPositions = outputs[0].Split("|").Select(x =>
	// 			new Vector3(float.Parse(x.Split("$")[0]), float.Parse(x.Split("$")[1]), float.Parse(x.Split("$")[2])))
	// 		.ToArray();
	// 	Vector3[] newEulers = outputs[1].Split("|").Select(x =>
	// 			new Vector3(float.Parse(x.Split("$")[0]), float.Parse(x.Split("$")[1]), float.Parse(x.Split("$")[2])))
	// 		.ToArray();
	// 	return newPositions.Zip(newEulers).Select(x => new Point(x.First, ToQ(x.Second))).ToArray();
	// }
	
	/*
	 * import sys

from scipy.interpolate import pchip_interpolate

if __name__ == '__main__':
	positions = list(map(lambda x: list(map(lambda y: float(y), x.split("$"))), sys.argv[1].split("|")))
	eulers = list(map(lambda x: list(map(lambda y: float(y), x.split("$"))), sys.argv[2].split("|")))
	amount = float(sys.argv[3])
	# print(f"{len(positions), len(positions[0]), amount}")
	x = range(-1, len(positions) - 1)
	inputs = [1/amount * x for x in range(int((len(positions)-2) * amount))]
	output_pos = pchip_interpolate(x, positions, inputs)
	output_rot = pchip_interpolate(x, eulers, inputs)
	pos_string = "|".join(map(lambda x: "$".join([str(i) for i in x]), output_pos))
	rot_string = "|".join(map(lambda x: "$".join([str(i) for i in x]), output_rot))
	print(pos_string + "^" + rot_string)
	 */
	
	static List<Point> ResampleCurve(Point[] originalPoints, int desiredPointCount)
	{
		List<Point> resampledPoints = new List<Point>();

		// Calculate the cumulative length of the curve.
		float[] cumulativeLengths = new float[originalPoints.Length];
		if (originalPoints.Length == 0)
		{
			return resampledPoints;
		}
		cumulativeLengths[0] = 0;
		float totalLength = 0;

		for (int i = 1; i < originalPoints.Length; i++)
		{
			float segmentLength = originalPoints[i - 1].GetDistance(originalPoints[i]);
			totalLength += segmentLength;
			cumulativeLengths[i] = totalLength;
		}

		// Determine the spacing between points.
		float targetSpacing = totalLength / (desiredPointCount - 1);

		// Resample the curve with evenly spaced points.
		for (int i = 0; i < desiredPointCount; i++)
		{
			float targetLength = i * targetSpacing;

			// Find the segment containing the target length.
			int segmentIndex = 0;
			while (segmentIndex < cumulativeLengths.Length - 1 && targetLength > cumulativeLengths[segmentIndex + 1])
			{
				segmentIndex++;
			}

			// Interpolate the point along the segment.
			float t = (targetLength - cumulativeLengths[segmentIndex]) /
					  (cumulativeLengths[segmentIndex + 1] - cumulativeLengths[segmentIndex]);
			Point interpolatedPoint = Interpolate(originalPoints[segmentIndex], originalPoints[segmentIndex + 1], t);

			resampledPoints.Add(interpolatedPoint);
		}

		return resampledPoints;
	}
			

	static Point Interpolate(Point p1, Point p2, float t)
	{
		Vector3 p = p1.position + (p2.position - p1.position) * t;
		Quaternion q = Quaternion.Slerp(p1.rotation, p2.rotation, t);
		return new Point(p, q);
	}

	public static Vector3 FromQ2(Quaternion q)
	{
		Vector3 euler;

		// if the input quaternion is normalized, this is exactly one. Otherwise, this acts as a correction factor for the quaternion's not-normalizedness
		float unit = (q.X * q.X) + (q.Y * q.Y) + (q.Z * q.Z) + (q.W * q.W);

		// this will have a magnitude of 0.5 or greater if and only if this is a singularity case
		float test = q.X * q.W - q.Y * q.Z;

		if (test > 0.4995f * unit) // singularity at north pole
		{
			euler.X = MathF.PI / 2;
			euler.Y = 2f * MathF.Atan2(q.Y, q.X);
			euler.Z = 0;
		}
		else if (test < -0.4995f * unit) // singularity at south pole
		{
			euler.X = -MathF.PI / 2;
			euler.Y = -2f * MathF.Atan2(q.Y, q.X);
			euler.Z = 0;
		}
		else // no singularity - this is the majority of cases
		{
			euler.X = MathF.Asin(2f * (q.W * q.X - q.Y * q.Z));
			euler.Y = MathF.Atan2(2f * q.W * q.Y + 2f * q.Z * q.X, 1 - 2f * (q.X * q.X + q.Y * q.Y));
			euler.Z = MathF.Atan2(2f * q.W * q.Z + 2f * q.X * q.Y, 1 - 2f * (q.Z * q.Z + q.X * q.X));
		}

		// all the math so far has been done in radians. Before returning, we convert to degrees...
		euler = euler / MathF.PI * 180f;

		//...and then ensure the degree values are between 0 and 360
		euler.X %= 360;
		euler.Y %= 360;
		euler.Z %= 360;

		if (euler.X < 0)
		{
			euler.X += 360;
		}

		if (euler.Y < 0)
		{
			euler.Y += 360;
		}
		
		if (euler.Z < 0)
		{
			euler.Z += 360;
		}
		
		return euler;
	}

	public static Quaternion ToQ(Vector3 euler)
	{
		float xOver2 = euler.X * MathF.PI / 180f * 0.5f;
		float yOver2 = euler.Y * MathF.PI / 180f * 0.5f;
		float zOver2 = euler.Z * MathF.PI / 180f * 0.5f;

		float sinXOver2 = MathF.Sin(xOver2);
		float cosXOver2 = MathF.Cos(xOver2);
		float sinYOver2 = MathF.Sin(yOver2);
		float cosYOver2 = MathF.Cos(yOver2);
		float sinZOver2 = MathF.Sin(zOver2);
		float cosZOver2 = MathF.Cos(zOver2);

		Quaternion result;
		result.X = cosYOver2 * sinXOver2 * cosZOver2 + sinYOver2 * cosXOver2 * sinZOver2;
		result.Y = sinYOver2 * cosXOver2 * cosZOver2 - cosYOver2 * sinXOver2 * sinZOver2;
		result.Z = cosYOver2 * cosXOver2 * sinZOver2 - sinYOver2 * sinXOver2 * cosZOver2;
		result.W = cosYOver2 * cosXOver2 * cosZOver2 + sinYOver2 * sinXOver2 * sinZOver2;

		return result;
	}

	public static string testShouldStay = "a";
	public static string testShouldStay2 = "a";
}
