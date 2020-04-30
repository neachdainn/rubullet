cmake_minimum_required(VERSION 3.0.0)
project(cbullet CXX)

# Add the Bullet libraries.
add_subdirectory(libbullet3)

# Create the library for the Bullet C API
add_library(cbullet
	libbullet3/examples/SharedMemory/plugins/stablePDPlugin/SpAlg.cpp
	libbullet3/examples/SharedMemory/plugins/stablePDPlugin/SpAlg.h
	libbullet3/examples/SharedMemory/plugins/stablePDPlugin/Shape.cpp
	libbullet3/examples/SharedMemory/plugins/stablePDPlugin/Shape.h
	libbullet3/examples/SharedMemory/plugins/stablePDPlugin/RBDUtil.cpp
	libbullet3/examples/SharedMemory/plugins/stablePDPlugin/RBDUtil.h
	libbullet3/examples/SharedMemory/plugins/stablePDPlugin/RBDModel.cpp
	libbullet3/examples/SharedMemory/plugins/stablePDPlugin/RBDModel.h
	libbullet3/examples/SharedMemory/plugins/stablePDPlugin/MathUtil.cpp
	libbullet3/examples/SharedMemory/plugins/stablePDPlugin/MathUtil.h
	libbullet3/examples/SharedMemory/plugins/stablePDPlugin/KinTree.cpp
	libbullet3/examples/SharedMemory/plugins/stablePDPlugin/KinTree.h
	libbullet3/examples/SharedMemory/plugins/stablePDPlugin/BulletConversion.cpp
	libbullet3/examples/SharedMemory/plugins/stablePDPlugin/BulletConversion.h
	libbullet3/examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.cpp
	libbullet3/examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp
	libbullet3/examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.h
	libbullet3/examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp
	libbullet3/examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h
	libbullet3/examples/SharedMemory/IKTrajectoryHelper.cpp
	libbullet3/examples/SharedMemory/IKTrajectoryHelper.h
	libbullet3/examples/ExampleBrowser/InProcessExampleBrowser.cpp
	libbullet3/examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp
	libbullet3/examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.h
	libbullet3/examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp
	libbullet3/examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.h
	libbullet3/examples/OpenGLWindow/SimpleCamera.cpp
	libbullet3/examples/OpenGLWindow/SimpleCamera.h
	libbullet3/examples/TinyRenderer/geometry.cpp
	libbullet3/examples/TinyRenderer/model.cpp
	libbullet3/examples/TinyRenderer/tgaimage.cpp
	libbullet3/examples/TinyRenderer/our_gl.cpp
	libbullet3/examples/TinyRenderer/TinyRenderer.cpp
	libbullet3/examples/SharedMemory/InProcessMemory.cpp
	libbullet3/examples/SharedMemory/PhysicsClient.cpp
	libbullet3/examples/SharedMemory/PhysicsClient.h
	libbullet3/examples/SharedMemory/PhysicsServer.cpp
	libbullet3/examples/SharedMemory/PhysicsServer.h
	libbullet3/examples/SharedMemory/GraphicsServerExample.cpp
	libbullet3/examples/SharedMemory/GraphicsClientExample.cpp
	libbullet3/examples/SharedMemory/RemoteGUIHelper.cpp
	libbullet3/examples/SharedMemory/GraphicsServerExample.h
	libbullet3/examples/SharedMemory/GraphicsClientExample.h
	libbullet3/examples/SharedMemory/RemoteGUIHelper.h
	libbullet3/examples/SharedMemory/GraphicsSharedMemoryCommands.h
	libbullet3/examples/SharedMemory/GraphicsSharedMemoryPublic.h
	libbullet3/examples/SharedMemory/PhysicsServerExample.cpp
	libbullet3/examples/SharedMemory/PhysicsServerExampleBullet2.cpp
	libbullet3/examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp
	libbullet3/examples/SharedMemory/PhysicsServerSharedMemory.cpp
	libbullet3/examples/SharedMemory/PhysicsServerSharedMemory.h
	libbullet3/examples/SharedMemory/PhysicsDirect.cpp
	libbullet3/examples/SharedMemory/PhysicsDirect.h
	libbullet3/examples/SharedMemory/PhysicsDirectC_API.cpp
	libbullet3/examples/SharedMemory/PhysicsDirectC_API.h
	libbullet3/examples/SharedMemory/PhysicsServerCommandProcessor.cpp
	libbullet3/examples/SharedMemory/PhysicsServerCommandProcessor.h
	libbullet3/examples/SharedMemory/b3PluginManager.cpp
	libbullet3/examples/SharedMemory/b3PluginManager.h

	libbullet3/examples/SharedMemory/PhysicsClientSharedMemory.cpp
	libbullet3/examples/SharedMemory/PhysicsClientSharedMemory.h
	libbullet3/examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp
	libbullet3/examples/SharedMemory/PhysicsClientSharedMemory_C_API.h

	libbullet3/examples/SharedMemory/PhysicsClientC_API.cpp
	libbullet3/examples/SharedMemory/PhysicsClientC_API.h
	libbullet3/examples/SharedMemory/Win32SharedMemory.cpp
	libbullet3/examples/SharedMemory/Win32SharedMemory.h
	libbullet3/examples/SharedMemory/PosixSharedMemory.cpp
	libbullet3/examples/SharedMemory/PosixSharedMemory.h
	libbullet3/examples/Utils/b3ResourcePath.cpp
	libbullet3/examples/Utils/b3ResourcePath.h
	libbullet3/examples/Utils/RobotLoggingUtil.cpp
	libbullet3/examples/Utils/RobotLoggingUtil.h

	libbullet3/examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp
	libbullet3/examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
	libbullet3/examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.h
	libbullet3/examples/ThirdPartyLibs/stb_image/stb_image.cpp
	libbullet3/examples/ThirdPartyLibs/stb_image/stb_image_write.cpp
	libbullet3/examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
	libbullet3/examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp
	libbullet3/examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
	libbullet3/examples/Importers/ImportMJCFDemo/BulletMJCFImporter.cpp
	libbullet3/examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp
	libbullet3/examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp
	libbullet3/examples/Importers/ImportURDFDemo/URDF2Bullet.cpp
	libbullet3/examples/Importers/ImportURDFDemo/UrdfParser.cpp
	libbullet3/examples/Importers/ImportURDFDemo/urdfStringSplit.cpp
	libbullet3/examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp 
	libbullet3/examples/MultiThreading/b3PosixThreadSupport.cpp
	libbullet3/examples/MultiThreading/b3Win32ThreadSupport.cpp
	libbullet3/examples/MultiThreading/b3ThreadSupportInterface.cpp
)

target_include_directories(cbullet
	PUBLIC
	${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/examples
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
)

target_link_libraries(cbullet
	PUBLIC
	BulletExampleBrowserLib
	BulletFileLoader
	BulletWorldImporter
	BulletSoftBody
	BulletDynamics
	BulletCollision
	BulletInverseDynamicsUtils
	BulletInverseDynamics
	LinearMath
	OpenGLWindow
	gwen
	BussIK
	Bullet3Common
	pthread
)

install(
	TARGETS cbullet
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
)
