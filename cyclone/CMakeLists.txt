cmake_minimum_required(VERSION 3.25)
project(cyclone)
set(cyclone_sources
	"core.cpp"
	"particle.cpp"
	"forceGenerators/particleForceGenerator.cpp"
	"forceGenerators/buoyancyForceGenerator.cpp"
	"forceGenerators/dragForceGenerator.cpp"
	"forceGenerators/springForceGenerator.cpp"
	"rigidbody.cpp"
)
set(cyclone_headers
	"precision.h"
	"core.h"
	"cyclone.h"
	"particle.h"
	"forceGenerators/particleForceGenerator.h"
	"forceGenerators/buoyancyForceGenerator.h"
	"forceGenerators/dragForceGenerator.h "
	"forceGenerators/springForceGenerator.h"
	"rigidbody.h"
		collider.cpp
		collider.h
		octree.cpp
		octree.h
)

set(cyclone_INC_DIR
 ${CMAKE_CURRENT_SOURCE_DIR}
 CACHE PATH "CYCLONE INCLUDE SOURCE PATH"
)
add_library(cyclone STATIC ${cyclone_sources} ${cyclone_headers})
target_include_directories(cyclone PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(cyclone PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
install (TARGETS cyclone DESTINATION lib)
install (FILES ${cyclone_headers} DESTINATION include/cyclone)
file(COPY ${cyclone_headers} DESTINATION "include")