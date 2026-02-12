file(GLOB HEADERS *.hpp)

file(GLOB SOURCES *.cpp)

add_library(fmac STATIC ${SOURCES} ${CMAKE_SOURCE_DIR}/3rdparty/gruenschloss/kuo-2d-proj/sobol.cpp ${CMAKE_SOURCE_DIR}/3rdparty/bessel/j1.c)

target_include_directories(fmac PUBLIC "${CMAKE_SOURCE_DIR}/include")
target_include_directories(fmac PUBLIC "${CMAKE_SOURCE_DIR}/3rdparty/gruenschloss/kuo-2d-proj")
target_include_directories(fmac PUBLIC "${CMAKE_SOURCE_DIR}/3rdparty/gruenschloss/halton")
target_include_directories(fmac PUBLIC "${CMAKE_SOURCE_DIR}/3rdparty/bessel")

target_link_libraries(fmac PUBLIC ${OpenCV_LIBS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(fmac PUBLIC OpenMP::OpenMP_CXX)
endif()

install(TARGETS fmac DESTINATION lib)

install(FILES ${HEADERS} DESTINATION include/${CMAKE_PROJECT_NAME})
