#include "KinematicGraph.h"

#include <queue>
#include <unordered_set>

namespace kinematic_core
{
int KinematicGraph::dofCount() const
{
	int n = 0;
	for (const KinematicJoint& j : joints)
	{
		if (j.qIndex >= 0 && j.motion.enabled)
		{
			++n;
		}
	}
	return n;
}

int KinematicGraph::linkIndexById(const std::string& id) const
{
	for (size_t i = 0; i < links.size(); ++i)
	{
		if (links[i].id == id)
		{
			return static_cast<int>(i);
		}
	}
	return -1;
}

bool KinematicGraph::validateTree(std::string* errMsg) const
{
	if (links.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty links";
		}
		return false;
	}
	if (rootLinkIdx < 0 || rootLinkIdx >= static_cast<int>(links.size()))
	{
		if (errMsg)
		{
			*errMsg = "invalid rootLinkIdx";
		}
		return false;
	}
	std::unordered_set<int> visited;
	std::queue<int> q;
	q.push(rootLinkIdx);
	visited.insert(rootLinkIdx);
	while (!q.empty())
	{
		const int parent = q.front();
		q.pop();
		for (const KinematicJoint& j : joints)
		{
			if (j.parentLinkIdx != parent)
			{
				continue;
			}
			if (j.childLinkIdx < 0 || j.childLinkIdx >= static_cast<int>(links.size()))
			{
				if (errMsg)
				{
					*errMsg = "invalid child link index";
				}
				return false;
			}
			if (visited.count(j.childLinkIdx))
			{
				if (errMsg)
				{
					*errMsg = "cycle detected";
				}
				return false;
			}
			visited.insert(j.childLinkIdx);
			q.push(j.childLinkIdx);
		}
	}
	if (static_cast<int>(visited.size()) != static_cast<int>(links.size()))
	{
		if (errMsg)
		{
			*errMsg = "disconnected links";
		}
		return false;
	}
	return true;
}

} // namespace kinematic_core
