from pathlib import Path

from pixi_build_ros.distro import Distro
from pixi_build_ros.ros_generator import convert_package_xml_to_catkin_package, package_xml_to_conda_requirements

def test_package_xml_to_recipe_config(package_xmls: Path):
    # Read content from the file in the test data directory
    package_xml_path = package_xmls / "demo_nodes_cpp.xml"
    package_content = package_xml_path.read_text(encoding='utf-8')
    package = convert_package_xml_to_catkin_package(package_content)

    distro = Distro("jazzy")
    requirements = package_xml_to_conda_requirements(package, distro)
    
    [print(bbuild) for bbuild in requirements.build]

def test_ament_cmake_package_xml_to_recipe_config(package_xmls: Path):
    # Read content from the file in the test data directory
    package_xml_path = package_xmls / "demos_action_tutorials_interfaces.xml"
    package_content = package_xml_path.read_text(encoding='utf-8')
    package = convert_package_xml_to_catkin_package(package_content)

    distro = Distro("noetic")
    requirements = package_xml_to_conda_requirements(package, distro)

    [print(build) for build in requirements.build]
