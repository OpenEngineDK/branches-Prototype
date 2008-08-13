// Tank.
// -------------------------------------------------------------------
// Copyright (C) 2007 (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _SeanTank_H_
#define _SeanTank_H_

#include "ITank.h"
#include <Core/IModule.h>
#include <Scene/ISceneNode.h>
#include <Scene/SceneNode.h>
#include <Scene/GeometryNode.h>
#include <Scene/TransformationNode.h>
#include "../ShotManager.h"
#include "../GunManager.h"

namespace OpenEngine {
    namespace Sound {
        class IMonoSound;
        class OpenALSoundSystem;
    }

	namespace Prototype {
            namespace Vehicles {
		using namespace OpenEngine::Scene;
		using namespace OpenEngine::Physics;
		using namespace OpenEngine::Core;

        using OpenEngine::Sound::IMonoSound;
        using OpenEngine::Sound::OpenALSoundSystem;

		class SeanTank : public ITank {
        private:
            IMonoSound* sound;
            OpenALSoundSystem* soundmgr;

		public:
			static GeometryNode* bodyModel;
			static GeometryNode* turretModel;
			static GeometryNode* gunModel;

			SeanTank(DynamicBody* body, OpenALSoundSystem* soundmgr);
			virtual ~SeanTank();

			virtual void Process(const float timeSinceLast);

			static void SetModel(GeometryNode* body, GeometryNode* turret, GeometryNode* gun);

            TransformationNode* GetCameraTransformationNode();
            TransformationNode* GetCameraRotateTransformationNode();
			TransformationNode* GetTankTransformationNode();
			TransformationNode* GetTurretTransformationNode();
			TransformationNode* GetTurretGunTransformationNode();

			DynamicBody* GetDynamicBody();
			void ShootGun(int i);
		};
            }
	}
}

#endif // _Tank_H_
