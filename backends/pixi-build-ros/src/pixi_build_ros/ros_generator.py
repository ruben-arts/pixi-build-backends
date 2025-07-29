"""
Python generator implementation using Python bindings.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, List, Any
from pixi_build_backend.types.generated_recipe import (
    GenerateRecipeProtocol,
    GeneratedRecipe,
)
from pixi_build_backend.types.intermediate_recipe import NoArchKind, Python, Script, ConditionalRequirements, ItemPackageDependency
from pixi_build_backend.types.platform import Platform
from pixi_build_backend.types.project_model import ProjectModelV1
from pixi_build_backend.types.python_params import PythonParams

from .build_script import BuildScriptContext, Installer, BuildPlatform
from .utils import extract_entry_points
from .utils import read_pyproject_toml, get_build_input_globs, get_editable_setting

from catkin_pkg.package import Package as CatkinPackage, parse_package_string
import os
import yaml
from itertools import chain
from pprint import pprint

@dataclass
class ROSBackendConfig:
    """ROS backend configuration."""

    noarch: Optional[bool] = None
    env: Optional[Dict[str, str]] = None
    debug_dir: Optional[Path] = None
    extra_input_globs: Optional[List[str]] = None

    def is_noarch(self) -> bool:
        """Whether to build a noarch package or a platform-specific package."""
        return self.noarch is None or self.noarch

    def get_debug_dir(self) -> Optional[Path]:
        """Get debug directory if set."""
        return self.debug_dir


class ROSGenerator(GenerateRecipeProtocol):
    """ROS recipe generator using Python bindings."""

    def generate_recipe(
        self,
        model: ProjectModelV1,
        config: Dict[str, Any],
        manifest_path: str,
        host_platform: Platform,
        python_params: Optional[PythonParams] = None,
    ) -> GeneratedRecipe:
        """Generate a recipe for a Python package."""
        backend_config: ROSBackendConfig = ROSBackendConfig(**config)

        manifest_root = Path(manifest_path).parent

        # Read package.xml
        package = read_package_xml(manifest_root)

        # Create base recipe from model
        generated_recipe = GeneratedRecipe.from_model(model, manifest_root)

        # Get recipe components
        recipe = generated_recipe.recipe
        requirements = recipe.requirements

        # Resolve requirements for the host platform
        resolved_requirements = requirements.resolve(host_platform)

        # Merge package requirements into the host requirements
        build_type = package.get_build_type()
        name = package.name
        version = package.version
        group_depends = package.group_depends


        # Determine installer (pip or uv)
        installer = Installer.determine_installer(resolved_requirements.host)
        installer_name = installer.package_name()

        # Add installer to host requirements if not present
        if installer_name not in resolved_requirements.host:
            from pixi_build_backend.pixi_build_backend import PyItemPackageDependency
            from pixi_build_backend.types.intermediate_recipe import ItemPackageDependency

            inner_dep = PyItemPackageDependency(installer_name)
            requirements.host.append(ItemPackageDependency._from_inner(inner_dep))

        # Add python to both host and run requirements if not present
        if "python" not in resolved_requirements.host:
            from pixi_build_backend.pixi_build_backend import PyItemPackageDependency
            from pixi_build_backend.types.intermediate_recipe import ItemPackageDependency

            inner_dep = PyItemPackageDependency("python")
            requirements.host.append(ItemPackageDependency._from_inner(inner_dep))
        if "python" not in resolved_requirements.run:
            from pixi_build_backend.pixi_build_backend import PyItemPackageDependency
            from pixi_build_backend.types.intermediate_recipe import ItemPackageDependency

            inner_dep = PyItemPackageDependency("python")
            requirements.run.append(ItemPackageDependency._from_inner(inner_dep))

        # Determine build platform
        build_platform = BuildPlatform.current()

        # Get editable setting
        editable = get_editable_setting(python_params)

        # Generate build script
        build_script_context = BuildScriptContext(
            installer=installer,
            build_platform=build_platform,
            editable=editable,
            manifest_root=manifest_root,
        )
        build_script_lines = build_script_context.render()

        # Determine noarch setting
        noarch_kind = NoArchKind.python() if backend_config.is_noarch() else None

        # Read pyproject.toml
        pyproject_manifest = read_pyproject_toml(manifest_root)

        # Extract entry points
        entry_points = extract_entry_points(pyproject_manifest)

        # Update recipe components
        recipe.build.python = Python(entry_points=entry_points)
        recipe.build.noarch = noarch_kind
        recipe.build.script = Script(
            content=build_script_lines,
            env=backend_config.env,
        )

        return generated_recipe

    def extract_input_globs_from_build(self, config: ROSBackendConfig, workdir: Path, editable: bool) -> List[str]:
        """Extract input globs for the build."""
        return get_build_input_globs(config, workdir, editable)


def get_package_xml_content(manifest_root: Path) -> str:
    """Read package.xml file from the manifest root."""
    package_xml_path = manifest_root / "package.xml"
    if not package_xml_path.exists():
        raise FileNotFoundError(f"package.xml not found at {package_xml_path}")

    with open(package_xml_path, 'r') as f:
        return f.read()

def convert_package_xml_to_catkin_package(package_xml_content: str) -> CatkinPackage:
    package_reading_warnings = None
    package_xml = parse_package_string(package_xml_content, package_reading_warnings)

    # Evaluate conditions in the package.xml
    # TODO: validate the need for dealing with configuration conditions
    package_xml.evaluate_conditions(os.environ)

    # print(f"Read package.xml: {package_xml.name} version {package_xml.version}")

    return package_xml

def rosdep_to_conda_package_name(dep_name: str, distro: str) -> List[str]:
    with open(Path(__file__).parent.parent.parent / "robostack.yaml") as f:
        # Parse yaml file into dict
        robostack_data = yaml.safe_load(f)

    if dep_name not in robostack_data:
        return [f"ros-{distro}-{dep_name.replace('_', '-')}"]

    conda_packages = robostack_data[dep_name].get("robostack", [])

    if isinstance(conda_packages, dict):
        # TODO: Handle different platforms
        conda_packages = conda_packages.get("linux", [])

    return conda_packages

def package_xml_to_conda_requirements(
    pkg: CatkinPackage,
    distro: str = "noetic",
) -> ConditionalRequirements:
    build_tool_deps = pkg.buildtool_depends
    build_tool_deps += pkg.buildtool_export_depends
    build_tool_deps = [d.name for d in build_tool_deps if d.evaluated_condition]

    build_deps = pkg.build_depends
    build_deps += pkg.build_export_depends
    build_deps = [d.name for d in build_deps if d.evaluated_condition]
    conda_build_deps = [rosdep_to_conda_package_name(dep, distro) for dep in build_deps]
    conda_build_deps = list(chain.from_iterable(conda_build_deps))

    run_deps = pkg.run_depends
    run_deps += pkg.exec_depends
    run_deps += pkg.build_export_depends
    run_deps += pkg.buildtool_export_depends
    run_deps = [d.name for d in run_deps if d.evaluated_condition]
    conda_run_deps = [rosdep_to_conda_package_name(dep, distro) for dep in run_deps]
    conda_run_deps = list(chain.from_iterable(conda_run_deps))

    build_requirements = [ItemPackageDependency(name) for name in conda_build_deps]
    run_requirements = [ItemPackageDependency(name) for name in conda_run_deps]

    cond = ConditionalRequirements()
    cond.host = build_requirements
    cond.build = build_requirements
    cond.run = run_requirements

    return cond
