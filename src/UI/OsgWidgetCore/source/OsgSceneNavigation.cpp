/// @file OsgSceneNavigation.cpp
/// @brief OsgSceneNavigation 实现

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "OsgScene.h"

#include <osgGA/EventQueue>
#include <osgGA/GUIEventAdapter>
#include <osgViewer/GraphicsWindow>
#include <osgViewer/Viewer>

void OsgScene::resetNavigationInputQueues()
{
	auto resetOne = [](osgGA::EventQueue* q)
	{
		if (!q)
		{
			return;
		}
		q->clear();
		if (osgGA::GUIEventAdapter* st = q->getCurrentEventState())
		{
			st->setButtonMask(0);
		}
	};
	if (m_viewer.valid())
	{
		resetOne(m_viewer->getEventQueue());
	}
	if (m_graphicsWindow.valid())
	{
		resetOne(m_graphicsWindow->getEventQueue());
	}
}
