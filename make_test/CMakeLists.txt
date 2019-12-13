cmake_minimum_required(VERSION 2.8)

set(MUJOCO_INCLUDE_DIRS 
   "$ENV{HOME}/.mujoco/mujoco200_linux/include"
)
set(MUJOCO_LIBRARY_DIRS
  "$ENV{HOME}/.mujoco/mujoco200_linux/bin"
)

include_directories(
	"./include"
	${MUJOCO_INCLUDE_DIRS}
)

set(MY_LINKER_LIBS "")

link_directories(
	$ENV{HOME}/.mujoco/mujoco200_linux/bin
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -fPIC -Wall")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(MUJOCO_LIBRARIES "glfw3" "mujoco200" "stdc++" "glew" "GL")

add_executable(zmp_test "zmp_test.c")
target_link_libraries(zmp_test ${MUJOCO_LIBRARIES} ${MY_LINKER_LIBS})
