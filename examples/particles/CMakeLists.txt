set(APP_NAME particles)

add_executable(${APP_NAME} main.cpp
        Unit.cpp
        Unit.h
        UnitManager.cpp
        UnitManager.h)
target_include_directories(${APP_NAME} PRIVATE ${raylib_INCLUDE_DIR} ${IMGUI_INC_DIR} ${RLIMGUI_INC_DIR})
target_link_libraries(${APP_NAME} PRIVATE cyclone raylib imgui rlImGui)

add_custom_target("copyAssets${APP_NAME}" ALL COMMAND ${CMAKE_COMMAND} -E copy_directory
${CMAKE_CURRENT_SOURCE_DIR}/assets/
${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets/)

add_dependencies(${APP_NAME} "copyAssets${APP_NAME}")