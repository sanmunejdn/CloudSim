/// @file AiConfigDefaults.cpp
/// @brief AiConfigDefaults 实现

#include "AiConfigDefaults.h"

#include "AiDomainTypes.h"

AiConfigDto defaultAiConfigDto()
{
	AiConfigDto cfg;
	cfg.hardwareProfile = QStringLiteral("vram_8gb");
	cfg.parserPriorityDefault = QStringList{QStringLiteral("rules"), QStringLiteral("local"), QStringLiteral("remote")};
	cfg.remoteLlm.enabled = false;
	cfg.router.mode = QStringLiteral("explicit_ui");

	AiDomainModelConfig mesh;
	mesh.id = AiDomainIds::meshCreate();
	mesh.model = QStringLiteral("qwen2.5:3b");
	mesh.parserPriority = QStringList{QStringLiteral("rules"), QStringLiteral("local")};

	AiDomainModelConfig compose;
	compose.id = AiDomainIds::meshCompose();
	compose.model = QStringLiteral("qwen2.5:3b");
	compose.parserPriority = QStringList{QStringLiteral("local"), QStringLiteral("remote")};

	AiDomainModelConfig geom;
	geom.id = AiDomainIds::geometryRecognize();
	geom.model = QStringLiteral("qwen2.5vl:3b");
	geom.multimodal = true;
	geom.parserPriority = QStringList{QStringLiteral("local")};
	geom.unloadOtherModelsBeforeInfer = true;

	AiDomainModelConfig traj;
	traj.id = AiDomainIds::trajectoryFeature();
	traj.model = QStringLiteral("qwen2.5:3b");
	traj.parserPriority = QStringList{QStringLiteral("rules"), QStringLiteral("local")};

	AiDomainModelConfig scene;
	scene.id = AiDomainIds::sceneOps();
	scene.model = QStringLiteral("qwen2.5:3b");
	scene.parserPriority = QStringList{QStringLiteral("rules"), QStringLiteral("local")};

	AiDomainModelConfig processFlow;
	processFlow.id = AiDomainIds::processFlow();
	processFlow.model = QStringLiteral("qwen2.5:3b");
	processFlow.parserPriority = QStringList{QStringLiteral("rules"), QStringLiteral("local"), QStringLiteral("remote")};

	AiDomainModelConfig designParts;
	designParts.id = AiDomainIds::designParts();
	designParts.model = QStringLiteral("qwen2.5:3b");
	designParts.parserPriority = QStringList{QStringLiteral("rules"), QStringLiteral("local")};

	cfg.domains = {mesh, compose, geom, traj, scene, processFlow, designParts};
	return cfg;
}
