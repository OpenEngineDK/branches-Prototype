SET( PROJECT_NAME "Prototype-Base")

# Project source code list
SET( PROTOTYPE_SOURCES
  # Add all the cpp source files here
  main.cpp
  GameFactory.cpp
  GameOrganizer.cpp
  ShotManager.cpp
  Crosshair.cpp
  ClassicMovementHandler.cpp
  TankManager.cpp
  TankController.cpp
  GunManager.cpp

  NetworkHandler.cpp

  # Vehicles
  Vehicles/SnowTank.cpp
  Vehicles/SeanTank.cpp

  # Weapons
  Weapons/LaserShot.cpp
  Weapons/RapidLaserShot.cpp
  Weapons/Landmine.cpp
  Weapons/Shield.cpp
  Weapons/LightningShot.cpp
  Weapons/LaserGun.cpp
  Weapons/RapidLaserGun.cpp
  Weapons/LandmineLayer.cpp
  Weapons/ShieldGenerator.cpp
  Weapons/LightningGun.cpp

  # Gamemodes
  Gamemodes/TestGamemode.cpp
  
  # Physics
  ForceHandler.cpp
  CustomRayCallback.cpp
  
  # Particle system
  ParticleGroupBuilder.cpp
  
  # Map loading
  MapLoader.cpp
)

# todo get rid of this!@#!
# Include if using SDL under Mac OS X
IF(APPLE)
  SET(PROTOTYPE_SOURCES ${PROTOTYPE_SOURCES} ${SDL_MAIN_FOR_MAC})
ENDIF(APPLE)

# Project executable
ADD_EXECUTABLE(Prototype-Base ${PROTOTYPE_SOURCES})

# Project dependencies
TARGET_LINK_LIBRARIES(Prototype-Base 
  OpenEngine_Core
  OpenEngine_Logging
  OpenEngine_Display
  OpenEngine_Devices
  OpenEngine_Renderers
  OpenEngine_Renderers_OpenGL
  OpenEngine_Resources
  OpenEngine_Scene
  # Extensions
  Extensions_AccelerationStructures
  Extensions_SDL
  Extensions_OpenGLRenderer
  Extensions_OBJResource
  Extensions_TGAResource
  Extensions_ColladaResource
  Extensions_ErlNetwork
  Extensions_RBPhysics
  Extensions_PropertyList
  Extensions_ParticleSystem
  Extensions_Sky
  Extensions_PostProcessing
  Extensions_PostProcessingEffects
  Extensions_OpenALSoundSystem
  Extensions_VorbisResource
)
