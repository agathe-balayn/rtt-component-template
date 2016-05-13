
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

include_directories(include
  ${RCI_INCLUDE_DIRS}
  ${RSC_INCLUDE_DIRS}
  ${RSB_INCLUDE_DIRS}
#  ${Eigen_INCLUDE_DIR}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${NemoMath_INCLUDE_DIRS}
  ${RTTToolchainTools_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${TINYXML_INCLUDE_DIRS}
  )

# Creates a component library libmyrobot-<target>.so
# and installs in the directory lib/orocos/myrobot/
#
orocos_component(RTTJointPositionController rtt-jpc.cpp rtt-jpc.hpp parsertools/KDLParser.cpp parsertools/KDLParser.hpp)
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/myrobot/ directory
#
# orocos_install_headers(rtt-controller-component.hpp)
#
# You may add multiple orocos_install_headers statements.
target_link_libraries(RTTJointPositionController ${RCI_LIBRARIES} ${RSC_LIBRARIES} ${RSB_LIBRARIES} ${NemoMath_LIBRARIES} ${RTTToolchainTools_LIBRARIES} ${orocos_kdl_LIBRARIES} ${TINYXML_LIBRARIES})
