/// @file DesEngine.cpp
/// @brief DES：派工 / 交期 / 故障 / 批量 / 装配汇合

#include "sim/DesEngine.h"

#include "sim/DispatchPolicies.h"

#include <QHash>
#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <random>
#include <vector>

namespace
{
enum class EvType
{
	JobRelease,
	EndOp,
	MachineFail,
	MachineRepair,
	SimEnd
};

struct Event
{
	double time = 0.0;
	long long seq = 0;
	EvType type = EvType::SimEnd;
	int jobId = -1;
	int machineNodeId = -1;
	int opSeq = -1;
};

struct EventLess
{
	bool operator()(const Event& a, const Event& b) const
	{
		if (a.time != b.time)
			return a.time > b.time;
		return a.seq > b.seq;
	}
};

struct JobState
{
	int id = 0;
	int templateIndex = 0;
	int nextOp = 0;
	bool active = false;
	bool completed = false;
	bool processing = false;
	bool blocked = false;
	int queuedMachine = -1;
	double enqueueTime = 0.0;
	double processStart = 0.0;
	double blockedSince = -1.0;
	double releaseTime = 0.0;
	double dueDateSec = 1e12;
	std::vector<int> batchPeers; // 同批一起加工的其它 job
};

struct MachineRuntime
{
	int nodeId = -1;
	QString title;
	int capacity = 1;
	int busy = 0;
	std::vector<int> queue;
	double busyTime = 0.0;
	double blockedTime = 0.0;
	double downTime = 0.0;
	double queueLenIntegral = 0.0;
	int maxQueue = 0;
	double lastUpdate = 0.0;
	double mtbfSec = 0.0;
	double mttrSec = 0.0;
	double busySinceRepair = 0.0;
	double downUntil = -1.0;
	bool failed = false;
	double failSince = -1.0;
	bool failScheduled = false;
};

struct SegmentBuffer
{
	double capacity = -1.0;
	int occupied = 0;
	int fullCount = 0;
	double invIntegral = 0.0;
	double maxInv = 0.0;
	double lastUpdate = 0.0;
};

void updateMachineIntegrals(MachineRuntime& m, double now)
{
	const double dt = now - m.lastUpdate;
	if (dt > 0.0)
	{
		m.queueLenIntegral += static_cast<double>(m.queue.size()) * dt;
		if (m.failed && m.failSince >= 0.0)
		{
			m.downTime += dt;
		}
		m.lastUpdate = now;
	}
	m.maxQueue = std::max(m.maxQueue, static_cast<int>(m.queue.size()));
}

void updateSegIntegrals(SegmentBuffer& b, double now)
{
	const double dt = now - b.lastUpdate;
	if (dt > 0.0)
	{
		b.invIntegral += static_cast<double>(b.occupied) * dt;
		b.lastUpdate = now;
	}
	b.maxInv = std::max(b.maxInv, static_cast<double>(b.occupied));
}

qint64 segKey(int fromId, int toId)
{
	return (static_cast<qint64>(fromId) << 32) ^ static_cast<qint64>(static_cast<quint32>(toId));
}

double remainingWork(const QVector<OpSpec>& ops, int fromOp)
{
	double s = 0.0;
	for (int i = fromOp; i < ops.size(); ++i)
	{
		const double b = std::max(1.0, ops[i].batchSize);
		s += ops[i].setupTimeSec + ops[i].processTimeSec * b;
	}
	return s;
}
} // namespace

DesEngine::DesEngine()
	: m_policy(std::make_unique<FifoPolicy>()), m_executor(std::make_unique<NullStationExecutor>())
{
}

void DesEngine::setDispatchPolicy(std::unique_ptr<IDispatchPolicy> policy)
{
	if (policy)
		m_policy = std::move(policy);
}

void DesEngine::setStationExecutor(std::unique_ptr<IStationExecutor> executor)
{
	if (executor)
		m_executor = std::move(executor);
}

SimStatistics DesEngine::run(const PlantGraph& plant, const JobSet& jobSet, double interarrivalSec,
							 const SimRunConfig& config, std::atomic_bool* cancelFlag)
{
	SimStatistics stats;
	stats.horizonSec = config.horizonSec;
	stats.warmupSec = config.warmupSec;
	if (jobSet.templates.isEmpty())
		return stats;

	QHash<int, MachineRuntime> machines;
	QHash<qint64, SegmentBuffer> segments;

	auto ensureMachine = [&](int nodeId)
	{
		if (machines.contains(nodeId))
			return;
		MachineRuntime m;
		m.nodeId = nodeId;
		const PlantNode n = plant.nodes.value(nodeId);
		m.title = n.title;
		m.capacity = std::max(1, static_cast<int>(std::floor(n.capacityQty > 0.0 ? n.capacityQty : 1.0)));
		m.mtbfSec = n.mtbfSec;
		m.mttrSec = n.mttrSec;
		machines.insert(nodeId, m);
	};

	for (const JobTemplate& tmpl : jobSet.templates)
	{
		for (int i = 0; i < tmpl.ops.size(); ++i)
		{
			ensureMachine(tmpl.ops[i].machineNodeId);
			MachineRuntime& mr = machines[tmpl.ops[i].machineNodeId];
			mr.mtbfSec = std::max(mr.mtbfSec, tmpl.ops[i].mtbfSec);
			mr.mttrSec = std::max(mr.mttrSec, tmpl.ops[i].mttrSec);
			if (i + 1 < tmpl.ops.size())
			{
				const qint64 key = segKey(tmpl.ops[i].machineNodeId, tmpl.ops[i + 1].machineNodeId);
				if (!segments.contains(key))
				{
					SegmentBuffer b;
					b.capacity = tmpl.ops[i].interBufferCapacity;
					segments.insert(key, b);
				}
			}
		}
	}

	std::priority_queue<Event, std::vector<Event>, EventLess> pq;
	long long seqGen = 0;
	auto pushEv = [&](double t, EvType type, int jobId = -1, int machineId = -1, int opSeq = -1)
	{
		Event e;
		e.time = t;
		e.seq = seqGen++;
		e.type = type;
		e.jobId = jobId;
		e.machineNodeId = machineId;
		e.opSeq = opSeq;
		pq.push(e);
	};

	const double horizon = std::max(0.0, config.horizonSec);
	pushEv(horizon, EvType::SimEnd);
	const double arrival = std::max(1e-6, interarrivalSec);
	const int plannedReleases = std::min(config.maxJobs, static_cast<int>(std::floor(horizon / arrival)) + 1);
	for (int i = 0; i < plannedReleases; ++i)
		pushEv(i * arrival, EvType::JobRelease, i + 1);

	QHash<int, JobState> jobs;
	int wip = 0;
	double wipIntegral = 0.0;
	double maxWip = 0.0;
	double lastWipT = 0.0;
	double makespan = 0.0;
	int completed = 0;
	int scrapped = 0;
	int released = 0;
	OperationTrace trace;
	std::mt19937 rng(config.seed);
	std::uniform_real_distribution<double> unit01(0.0, 1.0);

	auto bumpWip = [&](double now, int delta)
	{
		const double dt = now - lastWipT;
		if (dt > 0.0)
		{
			wipIntegral += static_cast<double>(wip) * dt;
			lastWipT = now;
		}
		wip += delta;
		maxWip = std::max(maxWip, static_cast<double>(wip));
	};

	auto opsOf = [&](const JobState& js) -> const QVector<OpSpec>&
	{ return jobSet.templates[js.templateIndex].ops; };

	auto scheduleFailIfNeeded = [&](MachineRuntime& m, double now)
	{
		if (m.mtbfSec <= 1e-9 || m.failed || m.failScheduled)
			return;
		const double remain = m.mtbfSec - m.busySinceRepair;
		if (remain <= 1e-9)
		{
			pushEv(now, EvType::MachineFail, -1, m.nodeId);
			m.failScheduled = true;
		}
		else if (m.busy > 0)
		{
			pushEv(now + remain, EvType::MachineFail, -1, m.nodeId);
			m.failScheduled = true;
		}
	};

	std::function<void(int, double)> tryDispatch;

	auto completeJob = [&](JobState& js, double now)
	{
		js.completed = true;
		js.active = false;
		js.processing = false;
		js.blocked = false;
		bumpWip(now, -1);
		completed += 1;
		makespan = std::max(makespan, now);
	};

	auto enqueueJobAtOp = [&](JobState& js, double now)
	{
		const QVector<OpSpec>& ops = opsOf(js);
		if (js.nextOp < 0 || js.nextOp >= ops.size())
			return;
		const int mid = ops[js.nextOp].machineNodeId;
		MachineRuntime& m = machines[mid];
		updateMachineIntegrals(m, now);
		m.queue.push_back(js.id);
		js.queuedMachine = mid;
		js.enqueueTime = now;
		js.processing = false;
		js.blocked = false;
		tryDispatch(mid, now);
	};

	auto tryLeaveMachine = [&](JobState& js, int finishedOp, int machineId, double now) -> bool
	{
		const QVector<OpSpec>& ops = opsOf(js);
		const int nextOp = finishedOp + 1;
		if (nextOp >= ops.size())
		{
			MachineRuntime& m = machines[machineId];
			m.busy = std::max(0, m.busy - 1);
			completeJob(js, now);
			for (int peer : js.batchPeers)
			{
				if (jobs.contains(peer) && jobs[peer].active)
					completeJob(jobs[peer], now);
			}
			js.batchPeers.clear();
			js.nextOp = nextOp;
			tryDispatch(machineId, now);
			return true;
		}
		const int nextMachine = ops[nextOp].machineNodeId;
		const qint64 key = segKey(ops[finishedOp].machineNodeId, nextMachine);
		if (segments.contains(key) && segments[key].capacity >= 0.0)
		{
			SegmentBuffer& b = segments[key];
			updateSegIntegrals(b, now);
			if (b.occupied >= static_cast<int>(b.capacity))
			{
				b.fullCount += 1;
				js.blocked = true;
				js.blockedSince = now;
				js.nextOp = finishedOp;
				js.processing = false;
				return false;
			}
			b.occupied += 1;
		}
		MachineRuntime& m = machines[machineId];
		m.busy = std::max(0, m.busy - 1);
		js.nextOp = nextOp;
		js.blocked = false;
		for (int peer : js.batchPeers)
		{
			if (!jobs.contains(peer))
				continue;
			JobState& pj = jobs[peer];
			if (!pj.active || pj.completed)
				continue;
			pj.nextOp = nextOp;
			pj.processing = false;
			enqueueJobAtOp(pj, now);
		}
		js.batchPeers.clear();
		enqueueJobAtOp(js, now);
		tryDispatch(machineId, now);
		return true;
	};

	tryDispatch = [&](int machineId, double now)
	{
		MachineRuntime& m = machines[machineId];
		updateMachineIntegrals(m, now);
		if (m.failed || (m.downUntil >= 0.0 && now < m.downUntil))
			return;

		while (m.busy < m.capacity && !m.queue.empty())
		{
			// 窥视队首确定 batch / assembly 需求
			const int headId = m.queue.front();
			if (!jobs.contains(headId))
			{
				m.queue.erase(m.queue.begin());
				continue;
			}
			JobState& head = jobs[headId];
			const QVector<OpSpec>& hops = opsOf(head);
			if (head.nextOp < 0 || head.nextOp >= hops.size())
			{
				m.queue.erase(m.queue.begin());
				continue;
			}
			const OpSpec& hop = hops[head.nextOp];
			const int need = std::max(1, static_cast<int>(std::floor(
									   hop.kind == QStringLiteral("assembly") ? hop.requiredInputs : hop.batchSize)));
			if (static_cast<int>(m.queue.size()) < need)
				break;

			DispatchContext ctx;
			ctx.machineNodeId = machineId;
			ctx.now = now;
			for (int jid : m.queue)
			{
				const JobState& js = jobs[jid];
				const QVector<OpSpec>& ops = opsOf(js);
				if (js.nextOp < 0 || js.nextOp >= ops.size())
					continue;
				if (ops[js.nextOp].machineNodeId != machineId)
					continue;
				ReadyOpCandidate c;
				c.jobId = jid;
				c.opSeq = js.nextOp;
				c.processTimeSec = ops[js.nextOp].setupTimeSec +
								   ops[js.nextOp].processTimeSec * std::max(1.0, ops[js.nextOp].batchSize);
				c.enqueueTime = js.enqueueTime;
				c.priority = ops[js.nextOp].priority;
				c.dueDateSec = js.dueDateSec;
				c.remainingWorkSec = remainingWork(ops, js.nextOp);
				ctx.candidates.append(c);
			}
			if (ctx.candidates.isEmpty())
				break;
			const int pick = m_policy ? m_policy->select(ctx) : 0;
			if (pick < 0 || pick >= ctx.candidates.size())
				break;
			const int jobId = ctx.candidates[pick].jobId;
			JobState& js = jobs[jobId];
			const QVector<OpSpec>& ops = opsOf(js);
			const int opSeq = js.nextOp;
			const OpSpec& op = ops[opSeq];
			const int batchNeed =
				std::max(1, static_cast<int>(std::floor(op.kind == QStringLiteral("assembly") ? op.requiredInputs
																							 : op.batchSize)));

			// 收集同机同工序同伴
			std::vector<int> peers;
			peers.push_back(jobId);
			for (int jid : m.queue)
			{
				if (jid == jobId)
					continue;
				JobState& ojs = jobs[jid];
				if (!ojs.active || ojs.nextOp != opSeq)
					continue;
				if (opsOf(ojs)[ojs.nextOp].machineNodeId != machineId)
					continue;
				peers.push_back(jid);
				if (static_cast<int>(peers.size()) >= batchNeed)
					break;
			}
			if (static_cast<int>(peers.size()) < batchNeed)
				break;

			for (int jid : peers)
			{
				auto qIt = std::find(m.queue.begin(), m.queue.end(), jid);
				if (qIt != m.queue.end())
					m.queue.erase(qIt);
			}
			updateMachineIntegrals(m, now);

			if (opSeq > 0)
			{
				const int prevMachine = ops[opSeq - 1].machineNodeId;
				const qint64 key = segKey(prevMachine, op.machineNodeId);
				if (segments.contains(key) && segments[key].capacity >= 0.0)
				{
					SegmentBuffer& b = segments[key];
					updateSegIntegrals(b, now);
					b.occupied = std::max(0, b.occupied - static_cast<int>(peers.size()));
				}
			}

			js.batchPeers.clear();
			for (size_t i = 1; i < peers.size(); ++i)
			{
				jobs[peers[i]].processing = true;
				jobs[peers[i]].queuedMachine = -1;
				if (op.kind == QStringLiteral("assembly"))
				{
					// 汇合：同伴并入 leader，不再单独前进
					jobs[peers[i]].active = false;
					jobs[peers[i]].completed = true;
					bumpWip(now, -1);
				}
				else
				{
					js.batchPeers.push_back(peers[i]);
				}
			}

			const double batchFactor = op.kind == QStringLiteral("assembly") ? 1.0 : static_cast<double>(peers.size());
			const double totalTime = op.setupTimeSec + op.processTimeSec * batchFactor;
			StationBinding binding;
			const double proc = m_executor->beginProcess(machineId, jobId, totalTime, binding);
			js.processing = true;
			js.queuedMachine = -1;
			js.processStart = now;
			m.busy += 1;
			m.busySinceRepair += proc;
			pushEv(now + std::max(0.0, proc), EvType::EndOp, jobId, machineId, opSeq);
			scheduleFailIfNeeded(m, now);
		}
	};

	auto retryBlocked = [&](double now)
	{
		QVector<int> ids;
		for (auto it = jobs.begin(); it != jobs.end(); ++it)
		{
			if (it.value().blocked && it.value().active && !it.value().completed)
				ids.append(it.key());
		}
		for (int jid : ids)
		{
			JobState& js = jobs[jid];
			if (!js.blocked)
				continue;
			const int finishedOp = js.nextOp;
			const QVector<OpSpec>& ops = opsOf(js);
			if (finishedOp < 0 || finishedOp >= ops.size())
				continue;
			const int machineId = ops[finishedOp].machineNodeId;
			const double blockedSince = js.blockedSince;
			if (tryLeaveMachine(js, finishedOp, machineId, now) && blockedSince >= 0.0)
			{
				machines[machineId].blockedTime += now - blockedSince;
				js.blockedSince = -1.0;
			}
		}
	};

	while (!pq.empty())
	{
		if (cancelFlag && cancelFlag->load())
			break;
		const Event ev = pq.top();
		pq.pop();
		const double now = ev.time;
		if (ev.type != EvType::SimEnd && now > horizon + 1e-9)
			continue;

		switch (ev.type)
		{
		case EvType::JobRelease:
		{
			if (released >= config.maxJobs)
				break;
			JobState js;
			js.id = ev.jobId;
			js.templateIndex = released % jobSet.templates.size();
			js.nextOp = 0;
			js.active = true;
			js.releaseTime = now;
			const JobTemplate& tmpl = jobSet.templates[js.templateIndex];
			double work = remainingWork(tmpl.ops, 0);
			js.dueDateSec = tmpl.dueDateSec >= 0.0 ? (now + tmpl.dueDateSec) : (now + work * 1.5);
			jobs.insert(js.id, js);
			released += 1;
			bumpWip(now, 1);
			enqueueJobAtOp(jobs[js.id], now);
			break;
		}
		case EvType::EndOp:
		{
			auto jit = jobs.find(ev.jobId);
			if (jit == jobs.end())
				break;
			JobState& js = jit.value();
			if (!js.processing || js.nextOp != ev.opSeq)
				break;
			MachineRuntime& m = machines[ev.machineNodeId];
			updateMachineIntegrals(m, now);
			const double start = js.processStart;
			if (now > config.warmupSec)
			{
				const double from = std::max(start, config.warmupSec);
				m.busyTime += now - from;
			}
			OperationTraceItem item;
			item.jobId = js.id;
			item.opSeq = ev.opSeq;
			item.machineNodeId = ev.machineNodeId;
			item.start = start;
			item.end = now;
			trace.items.append(item);

			const QVector<OpSpec>& ops = opsOf(js);
			const double scrapRate = (ev.opSeq >= 0 && ev.opSeq < ops.size()) ? ops[ev.opSeq].scrapRate : 0.0;
			if (scrapRate > 0.0 && unit01(rng) < scrapRate)
			{
				m.busy = std::max(0, m.busy - 1);
				js.processing = false;
				js.active = false;
				js.completed = true;
				bumpWip(now, -1);
				scrapped += 1;
				for (int peer : js.batchPeers)
				{
					if (jobs.contains(peer) && jobs[peer].active)
					{
						jobs[peer].active = false;
						jobs[peer].completed = true;
						bumpWip(now, -1);
						scrapped += 1;
					}
				}
				js.batchPeers.clear();
				tryDispatch(ev.machineNodeId, now);
				retryBlocked(now);
				break;
			}
			tryLeaveMachine(js, ev.opSeq, ev.machineNodeId, now);
			if (!js.blocked && js.blockedSince >= 0.0)
			{
				m.blockedTime += now - js.blockedSince;
				js.blockedSince = -1.0;
			}
			retryBlocked(now);
			break;
		}
		case EvType::MachineFail:
		{
			MachineRuntime& m = machines[ev.machineNodeId];
			updateMachineIntegrals(m, now);
			m.failed = true;
			m.failSince = now;
			m.failScheduled = false;
			m.busySinceRepair = 0.0;
			const double mttr = std::max(0.0, m.mttrSec);
			m.downUntil = now + mttr;
			pushEv(m.downUntil, EvType::MachineRepair, -1, m.nodeId);
			break;
		}
		case EvType::MachineRepair:
		{
			MachineRuntime& m = machines[ev.machineNodeId];
			updateMachineIntegrals(m, now);
			m.failed = false;
			m.failSince = -1.0;
			m.downUntil = -1.0;
			m.busySinceRepair = 0.0;
			tryDispatch(m.nodeId, now);
			break;
		}
		case EvType::SimEnd:
			goto finish;
		}
	}
finish:
	bumpWip(horizon, 0);
	for (auto it = machines.begin(); it != machines.end(); ++it)
		updateMachineIntegrals(it.value(), horizon);
	for (auto it = segments.begin(); it != segments.end(); ++it)
		updateSegIntegrals(it.value(), horizon);

	const double span = std::max(1e-9, horizon - config.warmupSec);
	stats.completedJobs = completed;
	stats.scrappedJobs = scrapped;
	stats.releasedJobs = released;
	stats.makespan = makespan;
	stats.throughputPerHour = completed * 3600.0 / span;
	stats.avgWip = wipIntegral / std::max(1e-9, horizon);
	stats.maxWip = maxWip;
	stats.trace = std::move(trace);

	double bestScore = -1.0;
	for (auto it = machines.begin(); it != machines.end(); ++it)
	{
		const MachineRuntime& m = it.value();
		MachineStat ms;
		ms.nodeId = m.nodeId;
		ms.title = m.title;
		ms.busyTimeSec = m.busyTime;
		ms.blockedTimeSec = m.blockedTime + m.downTime;
		ms.utilization = std::min(1.0, m.busyTime / (span * std::max(1, m.capacity)));
		ms.avgQueueLen = m.queueLenIntegral / std::max(1e-9, horizon);
		ms.maxQueueLen = m.maxQueue;
		stats.machines.append(ms);
		const double score = ms.utilization + (ms.blockedTimeSec > 1e-9 ? 0.15 : 0.0);
		if (score > bestScore)
		{
			bestScore = score;
			stats.bottleneckNodeId = ms.nodeId;
			stats.bottleneckTitle = ms.title;
		}
	}
	int segIndex = 0;
	for (auto it = segments.begin(); it != segments.end(); ++it, ++segIndex)
	{
		const SegmentBuffer& b = it.value();
		if (b.capacity < 0.0)
			continue;
		BufferStat bs;
		bs.nodeId = segIndex;
		bs.title = QStringLiteral("segment-%1").arg(segIndex);
		bs.avgInventory = b.invIntegral / std::max(1e-9, horizon);
		bs.maxInventory = b.maxInv;
		bs.fullCount = b.fullCount;
		stats.buffers.append(bs);
	}
	return stats;
}
