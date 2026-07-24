using System;
using System.Globalization;
using System.IO;
using System.Text.RegularExpressions;
using HslCommunication;
using HslCommunication.Robot.ABB;
using HslCommunication.Robot.FANUC;
using HslCommunication.Robot.KUKA;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace RobotCommBridge
{
	internal interface IBrandAdapter : IDisposable
	{
		bool Connect(ConnectRequest req, out string message);
		void Disconnect();
		bool GetFeedback(out FeedbackPayload payload, out string message);
	}

	internal sealed class ConnectRequest
	{
		public string Brand { get; set; }
		public string Host { get; set; } = "127.0.0.1";
		public int Port { get; set; }
		public string User { get; set; } = "Default User";
		public string Password { get; set; } = "robotics";
		public string MechUnit { get; set; } = "ROB_1";
		public string FanucPoseAddr { get; set; } = "D751";
		public string FanucJointAddr { get; set; } = "D777";
		public int FanucPoseLen { get; set; } = 6;
		public int FanucJointLen { get; set; } = 6;
		public string KukaJointVar { get; set; } = "$AXIS_ACT";
		public string KukaPoseVar { get; set; } = "$POS_ACT";
	}

	internal sealed class FeedbackPayload
	{
		public bool HasJoints { get; set; }
		public bool HasPose { get; set; }
		public double[] JointRad { get; set; } = Array.Empty<double>();
		public double[] PositionMm { get; set; } = new double[3];
		public double[] EulerDeg { get; set; } = new double[3];
		public string ControllerState { get; set; } = "";
		public long TimestampMs { get; set; }
	}

	internal static class UnitConvert
	{
		public static double DegToRad(double deg) => deg * Math.PI / 180.0;

		public static double[] DegArrayToRad(float[] deg)
		{
			if (deg == null || deg.Length == 0) return Array.Empty<double>();
			var r = new double[deg.Length];
			for (int i = 0; i < deg.Length; ++i) r[i] = DegToRad(deg[i]);
			return r;
		}

		public static double[] DegArrayToRad(double[] deg)
		{
			if (deg == null || deg.Length == 0) return Array.Empty<double>();
			var r = new double[deg.Length];
			for (int i = 0; i < deg.Length; ++i) r[i] = DegToRad(deg[i]);
			return r;
		}
	}

	/// <summary>Fanuc：对齐 HSLCLI D751 位姿 / D777 关节。</summary>
	internal sealed class FanucAdapter : IBrandAdapter
	{
		private FanucInterfaceNet _net;
		private ConnectRequest _cfg;

		public bool Connect(ConnectRequest req, out string message)
		{
			Disconnect();
			_cfg = req;
			int port = req.Port > 0 ? req.Port : 60008;
			_net = new FanucInterfaceNet(req.Host, port) { ConnectTimeOut = 3000 };
			var res = _net.ConnectServer();
			message = res.IsSuccess ? "ok" : res.Message;
			return res.IsSuccess;
		}

		public void Disconnect()
		{
			try { _net?.ConnectClose(); } catch { /* ignore */ }
			_net = null;
		}

		public bool GetFeedback(out FeedbackPayload payload, out string message)
		{
			payload = new FeedbackPayload
			{
				TimestampMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
			};
			message = "ok";
			if (_net == null)
			{
				message = "not connected";
				return false;
			}

			var joints = _net.ReadFloat(_cfg.FanucJointAddr, (ushort)_cfg.FanucJointLen);
			if (joints.IsSuccess && joints.Content != null && joints.Content.Length > 0)
			{
				payload.JointRad = UnitConvert.DegArrayToRad(joints.Content);
				payload.HasJoints = true;
			}

			var pose = _net.ReadFloat(_cfg.FanucPoseAddr, (ushort)_cfg.FanucPoseLen);
			if (pose.IsSuccess && pose.Content != null && pose.Content.Length >= 6)
			{
				payload.PositionMm[0] = pose.Content[0];
				payload.PositionMm[1] = pose.Content[1];
				payload.PositionMm[2] = pose.Content[2];
				payload.EulerDeg[0] = pose.Content[3];
				payload.EulerDeg[1] = pose.Content[4];
				payload.EulerDeg[2] = pose.Content[5];
				payload.HasPose = true;
			}

			if (!payload.HasJoints && !payload.HasPose)
			{
				message = "Fanuc read empty";
				return false;
			}
			return true;
		}

		public void Dispose() => Disconnect();
	}

	/// <summary>KUKA：KukaTcpNet（与 HSLCLI 一致）。</summary>
	internal sealed class KukaAdapter : IBrandAdapter
	{
		private KukaTcpNet _net;
		private ConnectRequest _cfg;

		public bool Connect(ConnectRequest req, out string message)
		{
			Disconnect();
			_cfg = req;
			int port = req.Port > 0 ? req.Port : 7000;
			_net = new KukaTcpNet(req.Host, port) { ConnectTimeOut = 3000 };
			var res = _net.ConnectServer();
			message = res.IsSuccess ? "ok" : res.Message;
			return res.IsSuccess;
		}

		public void Disconnect()
		{
			try { _net?.ConnectClose(); } catch { /* ignore */ }
			_net = null;
		}

		public bool GetFeedback(out FeedbackPayload payload, out string message)
		{
			payload = new FeedbackPayload
			{
				TimestampMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
			};
			message = "ok";
			if (_net == null)
			{
				message = "not connected";
				return false;
			}

			var axis = _net.ReadString(_cfg.KukaJointVar);
			if (axis.IsSuccess && !string.IsNullOrWhiteSpace(axis.Content))
			{
				if (TryParseKukaAxis(axis.Content, out var jointsDeg))
				{
					payload.JointRad = UnitConvert.DegArrayToRad(jointsDeg);
					payload.HasJoints = payload.JointRad.Length > 0;
				}
			}

			var pos = _net.ReadString(_cfg.KukaPoseVar);
			if (pos.IsSuccess && !string.IsNullOrWhiteSpace(pos.Content))
			{
				if (TryParseKukaPose(pos.Content, out var xyz, out var abc))
				{
					payload.PositionMm = xyz;
					payload.EulerDeg = abc;
					payload.HasPose = true;
				}
			}

			if (!payload.HasJoints && !payload.HasPose)
			{
				message = "KUKA read empty";
				return false;
			}
			return true;
		}

		// 例：{A1 10.0, A2 -20.0, ...}
		private static bool TryParseKukaAxis(string text, out double[] deg)
		{
			deg = Array.Empty<double>();
			var matches = Regex.Matches(text, @"A\d+\s+([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)", RegexOptions.IgnoreCase);
			if (matches.Count == 0)
			{
				matches = Regex.Matches(text, @"[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?");
			}
			if (matches.Count == 0) return false;
			var list = new System.Collections.Generic.List<double>();
			foreach (Match m in matches)
			{
				var token = m.Groups.Count > 1 && m.Groups[1].Success ? m.Groups[1].Value : m.Value;
				if (double.TryParse(token, NumberStyles.Float, CultureInfo.InvariantCulture, out var v))
					list.Add(v);
			}
			deg = list.ToArray();
			return deg.Length > 0;
		}

		// 例：{X 100, Y 0, Z 500, A 0, B 90, C 0}
		private static bool TryParseKukaPose(string text, out double[] xyz, out double[] abc)
		{
			xyz = new double[3];
			abc = new double[3];
			bool ok = false;
			ok |= TryNamed(text, "X", out xyz[0]);
			ok |= TryNamed(text, "Y", out xyz[1]);
			ok |= TryNamed(text, "Z", out xyz[2]);
			ok |= TryNamed(text, "A", out abc[0]);
			ok |= TryNamed(text, "B", out abc[1]);
			ok |= TryNamed(text, "C", out abc[2]);
			return ok;
		}

		private static bool TryNamed(string text, string name, out double value)
		{
			value = 0;
			var m = Regex.Match(text, name + @"\s+([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)", RegexOptions.IgnoreCase);
			if (!m.Success) return false;
			return double.TryParse(m.Groups[1].Value, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
		}

		public void Dispose() => Disconnect();
	}

	/// <summary>ABB RWS Web API。</summary>
	internal sealed class AbbAdapter : IBrandAdapter
	{
		private ABBWebApiClient _client;
		private ConnectRequest _cfg;

		public bool Connect(ConnectRequest req, out string message)
		{
			Disconnect();
			_cfg = req;
			int port = req.Port > 0 ? req.Port : 80;
			string user = string.IsNullOrEmpty(req.User) ? "Default User" : req.User;
			string pass = string.IsNullOrEmpty(req.Password) ? "robotics" : req.Password;
			try
			{
				_client = new ABBWebApiClient(req.Host, port, user, pass);
				var sys = _client.GetSystem();
				message = sys.IsSuccess ? "ok" : sys.Message;
				return sys.IsSuccess;
			}
			catch (Exception ex)
			{
				message = ex.Message;
				_client = null;
				return false;
			}
		}

		public void Disconnect()
		{
			_client = null;
		}

		public bool GetFeedback(out FeedbackPayload payload, out string message)
		{
			payload = new FeedbackPayload
			{
				TimestampMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
			};
			message = "ok";
			if (_client == null)
			{
				message = "not connected";
				return false;
			}

			string unit = string.IsNullOrEmpty(_cfg.MechUnit) ? "ROB_1" : _cfg.MechUnit;
			var joints = _client.GetJointTarget(unit);
			if (joints.IsSuccess && !string.IsNullOrWhiteSpace(joints.Content))
			{
				if (TryParseAbbJoints(joints.Content, out var deg))
				{
					payload.JointRad = UnitConvert.DegArrayToRad(deg);
					payload.HasJoints = payload.JointRad.Length > 0;
				}
			}

			var target = _client.GetRobotTarget(unit);
			if (target.IsSuccess && !string.IsNullOrWhiteSpace(target.Content))
			{
				if (TryParseAbbPose(target.Content, out var xyz, out var euler))
				{
					payload.PositionMm = xyz;
					payload.EulerDeg = euler;
					payload.HasPose = true;
				}
			}

			var state = _client.GetCtrlState();
			if (state.IsSuccess)
				payload.ControllerState = state.Content ?? "";

			if (!payload.HasJoints && !payload.HasPose)
			{
				message = "ABB read empty";
				return false;
			}
			return true;
		}

		private static bool TryParseAbbJoints(string xmlOrText, out double[] deg)
		{
			deg = Array.Empty<double>();
			var matches = Regex.Matches(xmlOrText, @"[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?");
			var list = new System.Collections.Generic.List<double>();
			foreach (Match m in matches)
			{
				if (double.TryParse(m.Value, NumberStyles.Float, CultureInfo.InvariantCulture, out var v))
					list.Add(v);
				if (list.Count >= 6) break;
			}
			deg = list.ToArray();
			return deg.Length > 0;
		}

		private static bool TryParseAbbPose(string text, out double[] xyz, out double[] euler)
		{
			xyz = new double[3];
			euler = new double[3];
			var matches = Regex.Matches(text, @"[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?");
			var nums = new System.Collections.Generic.List<double>();
			foreach (Match m in matches)
			{
				if (double.TryParse(m.Value, NumberStyles.Float, CultureInfo.InvariantCulture, out var v))
					nums.Add(v);
			}
			if (nums.Count < 6) return false;
			// 常见 robtarget：x y z + 四元数；无可靠四元数转欧拉时仅填位置
			xyz[0] = nums[0];
			xyz[1] = nums[1];
			xyz[2] = nums[2];
			if (nums.Count >= 7)
			{
				// 跳过四元数，姿态置 0（示教以关节为主）
			}
			return true;
		}

		public void Dispose() => Disconnect();
	}

	internal static class AdapterFactory
	{
		public static IBrandAdapter Create(string brand)
		{
			switch ((brand ?? "").Trim().ToLowerInvariant())
			{
				case "abb": return new AbbAdapter();
				case "kuka": return new KukaAdapter();
				case "fanuc":
				default: return new FanucAdapter();
			}
		}
	}
}
