project(eh_sim)
cmake_minimum_required(VERSION 3.1)
#add_definitions(-DDEBUG)
#add_definitions(-g)
#set(CMAKE_CXX_COMPILER      "g++" )
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS   "-g")
find_package(OpenCV REQUIRED)
add_subdirectory(utils)
aux_source_directory(. DIR_SRCS)
add_executable(t_main ${DIR_SRCS})
target_link_libraries(t_main UTILS ${OpenCV_LIBS})

# make irr
# g++ -I/home/jo/Downloads/irrlicht-1.8.4/include -I/usr/X11R6/include  -O3 -ffast-math graph.cpp -o graph -L /home/jo/Downloads/irrlicht-1.8.4/lib/Linux -lIrrlicht -L/usr/X11R6/lib -lGL -lXxf86vm -lXext -lX11 -lXcursor
