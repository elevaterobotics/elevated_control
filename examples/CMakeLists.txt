add_executable(config_verification config_verification.cpp)
target_link_libraries(config_verification PRIVATE ${PROJECT_NAME})
target_include_directories(config_verification PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(basic_usage basic_usage.cpp)
target_link_libraries(basic_usage PRIVATE ${PROJECT_NAME})
target_include_directories(basic_usage PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(velocity_commands velocity_commands.cpp)
target_link_libraries(velocity_commands PRIVATE ${PROJECT_NAME})
target_include_directories(velocity_commands PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(hand_guided hand_guided.cpp)
target_link_libraries(hand_guided PRIVATE ${PROJECT_NAME})
target_include_directories(hand_guided PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(spring_adjust spring_adjust.cpp)
target_link_libraries(spring_adjust PRIVATE ${PROJECT_NAME})
target_include_directories(spring_adjust PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})

set(EXAMPLE_CONFIG_YAMLS
  ${CMAKE_CURRENT_SOURCE_DIR}/joint_limits.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/elevate_config.yaml
)
foreach(exe config_verification basic_usage velocity_commands hand_guided spring_adjust)
  foreach(yaml ${EXAMPLE_CONFIG_YAMLS})
    add_custom_command(
      TARGET ${exe} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${yaml}
        $<TARGET_FILE_DIR:${exe}>
      VERBATIM
    )
  endforeach()
endforeach()

install(
	TARGETS config_verification basic_usage velocity_commands hand_guided spring_adjust
	RUNTIME DESTINATION examples
)

install(
	FILES
		elevate_config.yaml
		joint_limits.yaml
	DESTINATION examples
)
