set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

SET(CMAKE_BUILD_TYPE Debug)

set(COMMON
	Airplane.cpp
	Atmosphere.cpp
	ControlMap.cpp
	FGFDM.cpp
	Gear.cpp
	Glue.cpp
	Ground.cpp
	Hitch.cpp
	Hook.cpp
	Integrator.cpp
	Jet.cpp
	Launchbar.cpp
	Model.cpp
	PistonEngine.cpp
	PropEngine.cpp
	Propeller.cpp
	RigidBody.cpp
	Rotor.cpp
	Rotorpart.cpp
	SimpleJet.cpp
	Surface.cpp
	Thruster.cpp
	TurbineEngine.cpp
	Turbulence.cpp
	Wing.cpp
	fg_props.cpp
	)

set(SOURCES
	${COMMON}
	YASim.cxx
	fg_props.cxx
	FGGround.cpp
	)

add_library(yasim ${COMMON})
add_executable(yasim-test yasim-test.cpp)
add_executable(yasim-proptest proptest.cpp)
add_executable(yasim-svr yasim-svr.cpp)

set(SIMGEAR_CORE_LIBRARIES
	-lSimGearCore
	)

set(SIMGEAR_CORE_LIBRARY_DEPENDENCIES
	Threads::Threads
	-lexpat
	-lz
	)

target_link_libraries(yasim-test
		yasim
		${SIMGEAR_CORE_LIBRARIES}
		${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
		)

target_link_libraries(yasim-proptest
		yasim
		${SIMGEAR_CORE_LIBRARIES}
		${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
		)

target_link_libraries(yasim-svr
		yasim
		${SIMGEAR_CORE_LIBRARIES}
		${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
		)

#install(TARGETS yasim yasim-proptest RUNTIME DESTINATION bin)

