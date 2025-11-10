cmake_minimum_required(VERSION 3.25)
project(rlImGui)
set(RLIMGUI_SRC     
rlImGui.cpp     
)
set(RLIMGUI_HEADERS     
rlImGui.h
rlImGuiColors.h
imgui_impl_raylib.h
)
set(RLIMGUI_INC_DIR
 ${CMAKE_CURRENT_SOURCE_DIR}
 CACHE PATH "RLIMGUI INCLUDE SOURCE PATH"
)

add_library(rlImGui STATIC ${RLIMGUI_SRC} ${RLIMGUI_HEADERS})
target_link_libraries(rlImGui PRIVATE raylib imgui)
target_include_directories(rlImGui PUBLIC ${CMAKE_SOURCE_DIR} ${IMGUI_INC_DIR})
install (TARGETS rlImGui DESTINATION lib)
install (FILES ${RLIMGUI_HEADERS} DESTINATION include/rlimgui)
