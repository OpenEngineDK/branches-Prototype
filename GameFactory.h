// Game Factory for a test project.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

// header define to avoid including definitions several times
#ifndef _TEST_GAME_FACTORY_H_
#define _TEST_GAME_FACTORY_H_

// Game engine interface
#include <Core/IGameFactory.h>
#include <Display/Viewport.h>
#include <Scene/ISceneNode.h>

// HACK-EDY HACK HACK
//#include <Physics/FixedTimeStepPhysics.h>
#include "TankManager.h"
#include "ShotManager.h"
#include "ClassicMovementHandler.h"
#include <Display/FollowCamera.h>
#include <Physics/PhysicsFacade.h>

// Particle system
#include <Particles/ParticleSystem.h>

//forward references
namespace OpenEngine {
    namespace Sound {
        class OpenALSoundSystem;
    }
}

// Namespaces
using namespace OpenEngine::Core;
using namespace OpenEngine::Display;
using namespace OpenEngine::Renderers;
using namespace OpenEngine::Physics;
using namespace OpenEngine::Prototype;
using namespace OpenEngine::Particles;

// sound
using OpenEngine::Sound::OpenALSoundSystem;

/**
 * Game factory definition.
 *
 * The factory is implemented in GameFactory.cpp
 * Documentation on game factories can be found in Core/IGameFactory.h
 * 
 * @see IGameFactory
 */
class GameFactory : public IGameFactory {

private:
    IFrame*      frame;         // member variable for the engine frame
    IRenderer*   renderer;      // member variable for the engine renderer

    Viewport* viewport;

    //HACK ALSO
    ISceneNode* dynamicObjects;
    ISceneNode* staticObjects;
    ISceneNode* physicsObjects;
    ISceneNode* particleObjects;
    //FixedTimeStepPhysics* physic;
    PhysicsFacade* physics;
    ParticleSystem* particleSystem;
    OpenALSoundSystem* soundmgr;
public:
    GameFactory();
    bool         SetupEngine(IGameEngine& engine);
    IFrame*      GetFrame();
    IRenderer*   GetRenderer();

    //Half HACK
    ITank*       AddTank(int i/*, PhysicsFacade* physics*/);
    void 		 RemoveTank(int i);

    TankManager* tankMgr;
    ClassicMovementHandler* classicMovement;
    //FollowCamera* camera;
    FollowCamera* camera;
    ShotManager* shotMgr;
    ISceneNode* crosshairNode;
};

#endif // _TEST_GAME_FACTORY_H_
