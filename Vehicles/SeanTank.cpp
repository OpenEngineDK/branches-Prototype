// Tank.
// -------------------------------------------------------------------
// Copyright (C) 2007 (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#include "SeanTank.h"

#include <Sound/OpenALSoundSystem.h>
#include <Sound/ISound.h>
#include <Resources/ResourceManager.h>
#include <Resources/VorbisResource.h>

namespace OpenEngine {
	namespace Prototype {
            namespace Vehicles {
		using namespace OpenEngine::Math;

                using namespace OpenEngine::Resources;
		// initialize static members
		GeometryNode* SeanTank::bodyModel = NULL;
		GeometryNode* SeanTank::turretModel = NULL;
		GeometryNode* SeanTank::gunModel = NULL;

        SeanTank::SeanTank(DynamicBody* dynamicBody, OpenALSoundSystem* soundmgr) : ITank(dynamicBody), soundmgr(soundmgr) {
			GeometryNode* tankBody = dynamic_cast<GeometryNode*>(bodyModel->Clone());
			GeometryNode* tankTurret = dynamic_cast<GeometryNode*>(turretModel->Clone());
			GeometryNode* tankGun = dynamic_cast<GeometryNode*>(gunModel->Clone());
			this->dynamicBody = dynamicBody;

			tankBodyTrans = new TransformationNode();
			tankBodyTrans->AddNode(tankBody);

			cameraRotateTrans = new TransformationNode();
			cameraPivotTrans = new TransformationNode();
			cameraRotateTrans->AddNode(cameraPivotTrans);

			tankTurretTrans = new TransformationNode();
			tankTurretGunTrans = new TransformationNode();//
			tankTurretGunTrans->Move(0.0, 3.0, 0.0);
			tankTurretTrans->AddNode(tankTurretGunTrans);//
			tankTurretGunTrans->AddNode(tankGun);
			tankTurretTrans->AddNode(tankTurret);
			tankBodyTrans->AddNode(tankTurretTrans);//
			tankBodyTrans->AddNode(cameraRotateTrans);
			tankTurretTrans->Move(0.0, 9.0, 0.0);	

			position = Vector<3,float>();
			direction = Quaternion<float>();

			gunMgr = new GunManager(this);
			gunMgr->AddGun(new LaserGun(), GunManager::PRIMARY_WEAPON);
			gunMgr->AddGun(new RapidLaserGun(), GunManager::PRIMARY_WEAPON);
			gunMgr->AddGun(new LightningGun(), GunManager::PRIMARY_WEAPON);
			gunMgr->AddGun(new LandmineLayer(), GunManager::SECONDARY_WEAPON);
			gunMgr->AddGun(new ShieldGenerator(), GunManager::SECONDARY_WEAPON);

            // Add sound to the tank
            ISoundResourcePtr soundres = 
                ResourceManager<ISoundResource>::Create("splat.ogg");
            
            sound = soundmgr->CreateMonoSound(soundres);
            SoundNode* soundNode = new SoundNode(sound);
            tankTurretTrans->AddNode(soundNode);

		}

		SeanTank::~SeanTank() {
		}

		void SeanTank::Process(const float timeSinceLast) {
		}

		void SeanTank::SetModel(GeometryNode* body, GeometryNode* turret, GeometryNode* gun) {
			bodyModel = body;
			turretModel = turret;
			gunModel = gun;
		}

		TransformationNode* SeanTank::GetCameraTransformationNode() {
			return cameraPivotTrans;
		}

		TransformationNode* SeanTank::GetCameraRotateTransformationNode() {
			return cameraRotateTrans;
		}

		TransformationNode* SeanTank::GetTankTransformationNode() {
			return tankBodyTrans;
		}

		TransformationNode* SeanTank::GetTurretTransformationNode() {
			return tankTurretTrans;
		}

		TransformationNode* SeanTank::GetTurretGunTransformationNode() {
			return tankTurretGunTrans;
		}

		DynamicBody* SeanTank::GetDynamicBody() {
			return dynamicBody;
		}

		void SeanTank::ShootGun(int i) {
			gunMgr->ShootGun(i);
            sound->Play();
		}
        }
	}
}
