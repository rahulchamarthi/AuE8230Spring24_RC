execute_process(COMMAND "/home/rahul/Documents/git_ws/AuE8230Spring24_RC/catkin_ws/build/turtlebot3/turtlebot3_example/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/rahul/Documents/git_ws/AuE8230Spring24_RC/catkin_ws/build/turtlebot3/turtlebot3_example/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
