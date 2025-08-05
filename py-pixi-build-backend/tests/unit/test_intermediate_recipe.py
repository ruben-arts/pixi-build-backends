from pathlib import Path
from typing import Any
from pixi_build_backend.types.intermediate_recipe import IntermediateRecipe, ItemPackageDependency, Python


def test_from_yaml(snapshot: Any) -> None:
    yaml_file = Path(__file__).parent.parent / "data" / "boltons_recipe.yaml"
    yaml_content = yaml_file.read_text()

    recipe = IntermediateRecipe.from_yaml(yaml_content)

    assert snapshot == recipe.to_yaml()


def test_nested_setters() -> None:
    yaml_file = Path(__file__).parent.parent / "data" / "boltons_recipe.yaml"
    yaml_content = yaml_file.read_text()

    recipe = IntermediateRecipe.from_yaml(yaml_content)


    recipe.package.name = "new_package_name"


    assert str(recipe.package.name) == "new_package_name"


def test_intermediate_str(snapshot) -> None:
    yaml_file = Path(__file__).parent.parent / "data" / "boltons_recipe.yaml"
    yaml_content = yaml_file.read_text()

    recipe = IntermediateRecipe.from_yaml(yaml_content)


    assert str(recipe) == snapshot


def test_we_can_create_python() -> None:
    py = Python(["entry-point=module:function"])

    print(py)


def test_package_types() -> None:
    package = ItemPackageDependency("test")
    assert package.concrete.package_name() == "test"

    package = ItemPackageDependency("${{ compiler('c') }}")
    
    # assert str(package) == "${{ compiler('c') }}"

    assert package.template
    