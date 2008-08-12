#include "ForceHandler.h"
#include <Devices/Symbols.h>
#include <Math/Vector.h>
#include <Math/Quaternion.h>
#include <Logging/Logger.h>
#include <Renderers/IRenderer.h>
#include <Utils/Timer.h>

using namespace OpenEngine::Devices;
using namespace OpenEngine::Math;
using namespace OpenEngine::Logging;
using namespace OpenEngine::Physics;
using namespace OpenEngine::Renderers;
using namespace OpenEngine::Utils;
using namespace OpenEngine::Geometry;

ForceHandler::ForceHandler(DynamicBody * body, PhysicsFacade * physics) : 
  body(body),physics(physics),callback(new CustomRayCallback()),
  up(false),down(false),left(false),right(false)
{}



void ForceHandler::Handle(OpenEngine::Devices::KeyboardEventArg arg) {
  
  bool val = false;
  if (arg.type == KeyboardEventArg::PRESS) {
    val = true;
    switch(arg.sym) {
    case KEY_0:
      body->SetLinearDamping(body->GetLinearDamping()+1.0f);
      logger.info << "LinearDamping: " << body->GetLinearDamping() << logger.end;
      break;
    case KEY_9:
      body->SetLinearDamping(body->GetLinearDamping()-1.0f);
      logger.info << "LinearDamping: " << body->GetLinearDamping() << logger.end;
      break;
    case KEY_8:
      body->SetAngularDamping(body->GetAngularDamping()+1.0f);
      logger.info << "AngularDamping: " << body->GetAngularDamping() << logger.end;
      break;
    case KEY_7:
      body->SetAngularDamping(body->GetAngularDamping()-1.0f);
      logger.info << "AngularDamping: " << body->GetAngularDamping() << logger.end;
      break;
    case KEY_r:
      body->SetPosition(Vector<3,float>(0,30,5));
      body->SetRotation(Quaternion<float>(1,0,0,0));
      body->SetLinearVelocity(Vector<3,float>(0,0,0));
      body->SetAngularVelocity(Vector<3,float>(0,0,0));
      break;
    case KEY_c:
      {
        rNode.rayStart = body->GetPosition();
        rNode.rayEnd = body->GetPosition() + body->GetRotation().RotateVector( Vector<3,float>(1000,0,0) );

        callback->hasHit = false;
        callback->fraction = 100000000;
        logger.info << "Fire" << logger.end;
        physics->RayTest(rNode.rayStart,rNode.rayEnd,callback);
        
        if(callback->hasHit) {
          rNode.rayTime = Timer::GetTime() + 1000.0;
          rNode.rayEnd = rNode.rayStart + (rNode.rayEnd - rNode.rayStart)*callback->fraction;
          rNode.normal = callback->normal;

          if(typeid(DynamicBody) == typeid(*callback->body)) {
            DynamicBody & dynBody = *dynamic_cast<DynamicBody*>(callback->body);
            
            Vector<3,float> bodyPos = callback->body->GetPosition();
            Quaternion<float> invRot = callback->body->GetRotation().GetInverse();
          
            Vector<3,float> posObj = invRot.RotateVector(rNode.rayEnd -bodyPos);
            Vector<3,float> force = invRot.RotateVector( ( rNode.rayEnd -rNode.rayStart ).GetNormalize() * 5000);
            dynBody.ApplyForce(force, posObj);
          }
          
        }
      }
      break;
    default:
      break;
    }
  }
  //   else {
  //     logger.info << "rel" << logger.end;
  //   }

  switch(arg.sym) {
  case KEY_UP:
    up = val;
    break;
  case KEY_DOWN:
    down = val;
    break;
  case KEY_LEFT:
    left = val;
    break;
  case KEY_RIGHT:
    right = val;
    break;
  default:
    break;
  }
}


void ForceHandler::Initialize() {

}

void ForceHandler::Process(const float deltaTime, const float percent) {
  const Quaternion<float> & rot = body->GetRotation();
  Vector<3,float> acc = rot.RotateVector(Vector<3,float>(500,0,0));
  Vector<3,float> steer = rot.RotateVector(Vector<3,float>(0,1000,0));

  if(up) {
    body->ApplyForce(acc);
  }
  if(down) {
    body->ApplyForce(-acc);
  }
  if(left) {
     body->ApplyTorque(steer);
  }
  if(right) {
     body->ApplyTorque(-steer);
  }

}
void ForceHandler::Deinitialize() {

}
bool ForceHandler::IsTypeOf(const std::type_info& inf) {
  return typeid(ForceHandler) == inf;
}

OpenEngine::Renderers::IRenderNode * ForceHandler::GetRenderNode() {
  return &(this->rNode);
}

ForceHandler::RenderForceNode::RenderForceNode() :
  rayStart(Vector<3,float>(0,0,0)),
  rayEnd(Vector<3,float>(0,0,0)),
  rayTime(0.0)
{}

void ForceHandler::RenderForceNode::Apply(OpenEngine::Renderers::IRenderingView *view) {
  IRenderer & renderer = *view->GetRenderer();
  if(Timer::GetTime() < rayTime) {
    renderer.DrawLine(Line(rayStart,rayEnd),Vector<3,float>(1,0,0));

    renderer.DrawLine(Line(rayEnd,rayEnd+(normal.GetNormalize()*100)),Vector<3,float>(0,0,1));
  } 
}
