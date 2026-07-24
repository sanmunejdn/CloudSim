using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using HslCommunication;
using Newtonsoft.Json.Linq;

namespace RobotCommBridge
{
	internal static class Program
	{
		private static IBrandAdapter s_adapter;
		private static readonly object s_lock = new object();
		private static int s_listenPort = 19610;

		private static int Main(string[] args)
		{
			TryAuthorize();
			ParseArgs(args);

			var listener = new TcpListener(IPAddress.Any, s_listenPort);
			listener.Start();
			Console.WriteLine("[RobotCommBridge] listen 0.0.0.0:{0}", s_listenPort);
			Console.WriteLine("[RobotCommBridge] set HSL_AUTH_CODE for production; unlicensed ~8h limit");

			while (true)
			{
				var client = listener.AcceptTcpClient();
				ThreadPool.QueueUserWorkItem(_ => HandleClient(client));
			}
		}

		private static void ParseArgs(string[] args)
		{
			for (int i = 0; i < args.Length; ++i)
			{
				if ((args[i] == "--port" || args[i] == "-p") && i + 1 < args.Length
					&& int.TryParse(args[i + 1], out var p) && p > 0)
				{
					s_listenPort = p;
					++i;
				}
			}
		}

		private static void TryAuthorize()
		{
			string code = Environment.GetEnvironmentVariable("HSL_AUTH_CODE");
			if (string.IsNullOrWhiteSpace(code))
			{
				string path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "hsl.auth");
				if (File.Exists(path))
					code = File.ReadAllText(path).Trim();
			}

			if (!string.IsNullOrWhiteSpace(code))
			{
				bool ok = HslCommunication.Authorization.SetAuthorizationCode(code);
				Console.WriteLine(ok
					? "[RobotCommBridge] HSL authorization OK"
					: "[RobotCommBridge] HSL authorization FAILED — continuing (8h trial)");
			}
			else
			{
				Console.WriteLine("[RobotCommBridge] HSL unauthorized — 8h trial mode");
			}
		}

		private static void HandleClient(TcpClient client)
		{
			using (client)
			using (var stream = client.GetStream())
			using (var reader = new StreamReader(stream, Encoding.UTF8))
			using (var writer = new StreamWriter(stream, new UTF8Encoding(false)) { AutoFlush = true })
			{
				string line;
				while ((line = reader.ReadLine()) != null)
				{
					if (string.IsNullOrWhiteSpace(line)) continue;
					string resp = Dispatch(line);
					writer.WriteLine(resp);
				}
			}
		}

		private static string Dispatch(string line)
		{
			JObject req;
			try
			{
				req = JObject.Parse(line);
			}
			catch (Exception ex)
			{
				return Fail(0, "bad json: " + ex.Message);
			}

			int id = req.Value<int?>("id") ?? 0;
			string cmd = (req.Value<string>("cmd") ?? "").Trim().ToLowerInvariant();
			try
			{
				switch (cmd)
				{
					case "ping":
						return Ok(id, new JObject { ["pong"] = true });
					case "connect":
						return HandleConnect(id, req);
					case "disconnect":
						return HandleDisconnect(id);
					case "get_state":
						return HandleGetState(id);
					case "get_feedback":
						return HandleGetFeedback(id);
					default:
						return Fail(id, "unknown cmd: " + cmd);
				}
			}
			catch (Exception ex)
			{
				return Fail(id, ex.Message);
			}
		}

		private static string HandleConnect(int id, JObject req)
		{
			var cr = new ConnectRequest
			{
				Brand = req.Value<string>("brand") ?? "fanuc",
				Host = req.Value<string>("host") ?? "127.0.0.1",
				Port = req.Value<int?>("port") ?? 0,
				User = req.Value<string>("user") ?? "",
				Password = req.Value<string>("password") ?? "",
				MechUnit = req.Value<string>("mechUnit") ?? "ROB_1",
				FanucPoseAddr = req.Value<string>("fanucPoseAddr") ?? "D751",
				FanucJointAddr = req.Value<string>("fanucJointAddr") ?? "D777",
				FanucPoseLen = req.Value<int?>("fanucPoseLen") ?? 6,
				FanucJointLen = req.Value<int?>("fanucJointLen") ?? 6,
				KukaJointVar = req.Value<string>("kukaJointVar") ?? "$AXIS_ACT",
				KukaPoseVar = req.Value<string>("kukaPoseVar") ?? "$POS_ACT",
			};

			lock (s_lock)
			{
				s_adapter?.Dispose();
				s_adapter = AdapterFactory.Create(cr.Brand);
				if (!s_adapter.Connect(cr, out var msg))
				{
					s_adapter.Dispose();
					s_adapter = null;
					return Fail(id, msg);
				}
			}
			return Ok(id, new JObject { ["brand"] = cr.Brand });
		}

		private static string HandleDisconnect(int id)
		{
			lock (s_lock)
			{
				s_adapter?.Disconnect();
				s_adapter?.Dispose();
				s_adapter = null;
			}
			return Ok(id, new JObject());
		}

		private static string HandleGetState(int id)
		{
			bool connected;
			lock (s_lock) { connected = s_adapter != null; }
			return Ok(id, new JObject { ["robotConnected"] = connected });
		}

		private static string HandleGetFeedback(int id)
		{
			FeedbackPayload fb;
			string msg;
			lock (s_lock)
			{
				if (s_adapter == null)
					return Fail(id, "robot not connected");
				if (!s_adapter.GetFeedback(out fb, out msg))
					return Fail(id, msg);
			}

			var pose = new JObject
			{
				["positionMm"] = new JArray(fb.PositionMm[0], fb.PositionMm[1], fb.PositionMm[2]),
				["eulerDeg"] = new JArray(fb.EulerDeg[0], fb.EulerDeg[1], fb.EulerDeg[2]),
			};
			return Ok(id, new JObject
			{
				["hasJoints"] = fb.HasJoints,
				["hasPose"] = fb.HasPose,
				["jointRad"] = new JArray(fb.JointRad),
				["toolPoseInBase"] = pose,
				["controllerState"] = fb.ControllerState ?? "",
				["timestampMs"] = fb.TimestampMs,
			});
		}

		private static string Ok(int id, JObject extra)
		{
			extra["ok"] = true;
			extra["id"] = id;
			return extra.ToString(Newtonsoft.Json.Formatting.None);
		}

		private static string Fail(int id, string message)
		{
			return new JObject
			{
				["ok"] = false,
				["id"] = id,
				["message"] = message ?? "error",
			}.ToString(Newtonsoft.Json.Formatting.None);
		}
	}
}
