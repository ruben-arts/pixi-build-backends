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
from pixi_build_backend.types.intermediate_recipe import Script, ConditionalRequirements, Package

from pixi_build_backend.types.item import ItemPackageDependency
from pixi_build_backend.types.platform import Platform
from pixi_build_backend.types.project_model import ProjectModelV1
from pixi_build_backend.types.python_params import PythonParams

from .build_script import BuildScriptContext, BuildPlatform
from .distro import Distro
from .utils import get_build_input_globs, package_xml_to_conda_requirements, convert_package_xml_to_catkin_package, \
    get_package_xml_content


@dataclass
class ROSBackendConfig:
    """ROS backend configuration."""

    noarch: Optional[bool] = None
    env: Optional[Dict[str, str]] = None
    debug_dir: Optional[Path] = None
    extra_input_globs: Optional[List[str]] = None
    distro: Optional[str] = None

    def is_noarch(self) -> bool:
        """Whether to build a noarch package or a platform-specific package."""
        return self.noarch is None or self.noarch

    def get_debug_dir(self) -> Optional[Path]:
        """Get debug directory if set."""
        return self.debug_dir


def merge_requirements(model_requirements: ConditionalRequirements, package_requirements: ConditionalRequirements) -> ConditionalRequirements:
    """Merge two sets of requirements."""
    merged = ConditionalRequirements()

    # The model requirements are the base, coming from the pixi manifest
    # We need to only add the names for non existing dependencies
    def merge_unique_items(
        model: List[ItemPackageDependency],
        package: List[ItemPackageDependency],
    ) -> List[ItemPackageDependency]:
        """Merge unique items from source into target."""
        result = model
        # assert False, f"PACKAGE IS {package} type {type(package)} MODEL IS {model} type {type(model)}"

        # assert False, f"package type {type(package[0])}"
        for item in package:
            # assert False, f"PN {pn} type {type(pn)}"

            package_names = [i.concrete.package_name for i in model if i.concrete]

            # assert False, f"package names {package_names} type {type(package_names)}"
            # pn = item.concrete.package_name

            # assert False, f"BEFORE IF item {item} type {type(item)}"
            if item.concrete is not None and item.concrete.package_name not in package_names:
                # assert False, "BEFORE APPEND"
                result.append(item)
                # assert False, "BEFORE APPEND"
            if str(item.template) not in [str(i.template) for i in model]:
                result.append(item)
        # assert False, f"RESULT IS {result} type {type(result)}"
        return result

    merged.host = merge_unique_items(model_requirements.host, package_requirements.host)
    merged.build = merge_unique_items(model_requirements.build, package_requirements.build)
    merged.run = merge_unique_items(model_requirements.run, package_requirements.run)

    # If the dependency is of type Source in one of the requirements, we need to set them to Source for all variants

    return merged


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

        manifest_root = Path(manifest_path)

        # Read package.xml
        package_xml_str = get_package_xml_content(manifest_root)
        package_xml = convert_package_xml_to_catkin_package(package_xml_str)

        name = package_xml.name
        version = package_xml.version

        # TODO: Set this on the recipe
        package = Package(name, version)


        # Get requirements from package.xml
        distro = Distro(backend_config.distro)
        package_requirements = package_xml_to_conda_requirements(package_xml, distro)

        # Add standard dependencies
        build_deps = ["ninja", "python", "setuptools", "git", "git-lfs", "cmake", "cpython"]
        if host_platform.is_unix:
            build_deps.extend(["patch", "make", "coreutils"])
        if host_platform.is_windows:
            build_deps.extend(["m2-patch"])
        if host_platform.is_osx:
            build_deps.extend(["tapi"])

        build = package_requirements.build
        # assert False, "HERE I FAIL before I set build requirements"
        for dep in build_deps:
            build.append(ItemPackageDependency(name=dep))

        # Add compiler dependencies
        build.append(ItemPackageDependency("${{ compiler('c') }}"))
        build.append(ItemPackageDependency("${{ compiler('cxx') }}"))
        # assert False, "HERE I FAIL before setting build requirements"
        
        package_requirements.build = build

        # assert False, "HERE I FAIL before host deps"

        host_deps = ["python", "numpy", "pip", "pkg-config"]

        # assert False, "HERE I FAIL before getting host property"
        host = package_requirements.host
        for dep in host_deps:
            host.append(ItemPackageDependency(name=dep))
        # assert False, "HERE I FAIL before setting hst requirements"
        package_requirements.host = host

        # Create base recipe from model
        # assert False, "HERE I FAIL before frommodel"
        generated_recipe = GeneratedRecipe.from_model(model, manifest_root)

        # Get recipe components
        recipe = generated_recipe.recipe
        # recipe.package.name = name
        # recipe.package.version = version

        # Merge package requirements into the model requirements
        # assert "HERE I FAIL before merge requiremets"
        requirements = merge_requirements(recipe.requirements, package_requirements)
        recipe.requirements = requirements
        generated_recipe.recipe = recipe

        # Determine build platform
        build_platform = BuildPlatform.current()

        # Generate build script
        build_script_context = BuildScriptContext.load_from_template(package_xml, build_platform)
        build_script_lines = build_script_context.render()

        # Update recipe components
        # recipe.build.script = Script(
        #     content=build_script_lines,
        #     env=backend_config.env,
        # )

        # Stupid setter chain as it doesn't work directly
        script = Script(
            content=build_script_lines,
            env=backend_config.env,
            )        
        build = recipe.build
        build.script = script
        recipe.build = build
        generated_recipe.recipe = recipe

        # Test the build script before running to early out.
        assert generated_recipe.recipe.build.script.content == build_script_lines, recipe.build.script.content
        # raise RuntimeError(f"Generated recipe: {recipe.to_yaml()}")
        return generated_recipe

    def extract_input_globs_from_build(self, config: ROSBackendConfig, editable: bool) -> List[str]:
        """Extract input globs for the build."""
        return get_build_input_globs(config, editable)



