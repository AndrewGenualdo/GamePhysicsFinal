cmake_minimum_required(VERSION 3.25)
project(imgui)
set(IMGUI_SRC     
imgui.cpp     
imgui_draw.cpp     
imgui_tables.cpp     
imgui_widgets.cpp     
imgui_demo.cpp 
)
set(IMGUI_HEADERS     
imgui.h
imgui_internal.h
imconfig.h
imstb_truetype.h
imstb_rectpack.h
imstb_textedit.h
)
set(IMGUI_INC_DIR
 ${CMAKE_CURRENT_SOURCE_DIR}
 CACHE PATH "IMGUI INCLUDE SOURCE PATH"
)
add_library(imgui STATIC ${IMGUI_SRC} ${IMGUI_HEADERS})
target_include_directories(imgui PUBLIC ${CMAKE_SOURCE_DIR})
install (TARGETS imgui DESTINATION lib)
install (FILES ${IMGUI_HEADERS} DESTINATION include/imgui)
