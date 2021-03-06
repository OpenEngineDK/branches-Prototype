// Rigid body rendering.
// -------------------------------------------------------------------
// Copyright (C) 2007 (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _TESTGAMEMODE_H_
#define _TESTGAMEMODE_H_

#include "TemplatedGamemode.h"

namespace OpenEngine {
	namespace Prototype {
		namespace Gamemode {

			using namespace std;
			using namespace OpenEngine::Core;

			class TestGamemode : public TemplatedGamemode<TestGamemode> {

			public:
				int count;

				TestGamemode() {
					SetupCallbackHandler(this);
				};

				virtual ~TestGamemode() {};

				int OnGameModeInit();
				int OnGameModeExit();
				int OnGameModeStart();
				int OnGameModeEnd();
				int OnGameLoopProcess(float timeSinceLast);
				int OnPlayerConnect(int playerid);
				int OnPlayerDisconnect(int playerid);
				int OnPlayerRequestClass(int playerid, int classid);
				int OnPlayerSpawn(int playerid);
				int OnPlayerDeath(int playerid, int killerid, int reason);
				int OnPlayerEnterZone(int playerid, int zoneid);
				int OnPlayerLeaveZone(int playerid, int zoneid);
				int OnPlayerCommandText(int playerid, string cmdtext);

				int TestCallbackFunction();
				int TestCallbackFunction2();
				int TestCallbackFunction3();
			};
		}
	} // NS Prototype
} // NS OpenEngine

#endif
