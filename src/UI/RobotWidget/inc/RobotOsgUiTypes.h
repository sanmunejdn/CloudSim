#pragma once



#include <osg/Matrixd>

#include <osg/Vec3f>

#include <string>

#include <vector>



namespace RobotOsgUi

{



struct InstructionPoseAxis

{

	osg::Vec3f positionMm;

	osg::Vec3f eulerDeg;

	bool lineMotion = false;

	bool reachable = true;

	std::string robotBackendId;

	bool mountTcpOnPatRoot = false;

	bool hasLocalMatrix = false;

	double localMatrix[16]{};

	std::string urdfTcpAttachLinkName;

	osg::Matrixd worldMatrix;

	std::string backendId;

};



struct RawTrajectoryOverlayVertex

{

	osg::Vec3f positionMm;

	bool reachable = true;

};



struct RawTrajectoryOverlayFrame

{

	osg::Vec3f positionMm;

	osg::Vec3f eulerDeg;

	bool reachable = true;

};



struct RawTrajectoryPreviewOptions

{

	bool showAxes = true;

	int axisInterval = 0;

	int maxAxes = 50;

};



struct FeatureCatalogOverlayItem

{

	int displayIndex = 0;

	osg::Vec3f anchorWorldMm;

	osg::Vec3f labelWorldMm;

	bool hasEdgeSegment = false;

	osg::Vec3f edgeAWorldMm;

	osg::Vec3f edgeBWorldMm;

};



struct RobotFrameOverlayUpdate

{

	std::string robotRootBackendId;

	bool showToolFrames = false;

	struct ToolEntry

	{

		std::string name;

		std::string mountBackendId;

		osg::Matrixd localMatrix;

		bool active = false;

	};

	std::vector<ToolEntry> toolFrames;

	bool showUserFrames = false;

	struct UserEntry

	{

		std::string name;

		std::string mountBackendId;

		osg::Matrixd localMatrix;

	};

	std::vector<UserEntry> userFrames;

};



} // namespace RobotOsgUi

