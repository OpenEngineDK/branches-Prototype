// Rigid body rendering.
// -------------------------------------------------------------------
// Copyright (C) 2007 (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _LIGHTNINGGUN_H_
#define _LIGHTNINGGUN_H_

#include "IGun.h"

namespace OpenEngine {
  namespace Prototype {

	  class LightningGun : public IGun {

    public:
      LightningGun();

      virtual ~LightningGun();

      void ShootGun(ShotPosAndDir posAndDir);

	  bool GunReady();

    };

  } // NS Utils
} // NS OpenEngine

#endif // _DEFAULT_RIGID_BODY_RENDER_NODE_H_
