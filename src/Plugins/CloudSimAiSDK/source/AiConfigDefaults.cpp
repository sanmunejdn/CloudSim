#include "AiConfigDefaults.h"

#include "AiDomainTypes.h"

AiConfigDto defaultAiConfigDto()
{
	AiConfigDto cfg;
	cfg.hardwareProfile = QStringLiteral("vram_8gb");
	cfg.parserPriorityDefault = QStringList{
		QStringLiteral("rules"),
		QStringLiteral("local"),
		QStringLiteral("remote")
	};
	cfg.remoteLlm.enabled = false;
	cfg.router.mode = QStringLiteral("explicit_ui");

	AiDomainModelConfig mesh;
	mesh.id = AiDomainIds::meshCreate();
	mesh.model = QStringLiteral("qwen2.5:3b");
	mesh.parserPriority = QStringList{ QStringLiteral("rules"), QStringLiteral("local") };

	AiDomainModelConfig geom;
	geom.id = AiDomainIds::geometryRecognize();
	geom.model = QStringLiteral("qwen2.5vl:3b");
	geom.multimodal = true;
	geom.parserPriority = QStringList{ QStringLiteral("local") };
	geom.unloadOtherModelsBeforeInfer = true;

	cfg.domains = { mesh, geom };
	return cfg;
}
