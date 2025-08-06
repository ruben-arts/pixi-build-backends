from pathlib import Path
from typing import Any
from pixi_build_backend.types.conditional import ConditionalPackageDependency, ListOrItemPackageDependency
from pixi_build_backend.types.generated_recipe import GeneratedRecipe
from pixi_build_backend.types.item import ItemPackageDependency
from pixi_build_backend.types.project_model import ProjectModelV1


def test_generated_recipe_from_model(snapshot: Any) -> None:
    """Test initialization of ProjectModelV1."""
    model = ProjectModelV1(name="test_project", version="1.0.0")

    generated_recipe = GeneratedRecipe.from_model(model, Path("."))

    print(type(generated_recipe.recipe))

    assert snapshot == generated_recipe.recipe.to_yaml()


def test_setting_package_name_from_generated_recipe() -> None:
    """Test initialization of ProjectModelV1."""
    model = ProjectModelV1(name="test_project", version="1.0.0")

    generated_recipe = GeneratedRecipe.from_model(model, Path("."))

    generated_recipe.recipe.package.name = "new_package_name"
    assert str(generated_recipe.recipe.package.name) == "new_package_name"


def test_package_dependency_modification() -> None:
    """Test initialization of ProjectModelV1."""
    model = ProjectModelV1(name="test_project", version="1.0.0")

    generated_recipe = GeneratedRecipe.from_model(model, Path("."))

    generated_recipe.recipe.requirements.build.append(ItemPackageDependency("test_package"))
    generated_recipe.recipe.requirements.build.append(ItemPackageDependency("test_package"))
    
    assert len(generated_recipe.recipe.requirements.build) == 2



def test_conditional_item() -> None:
    """Test setting of conditional item."""
    
    conditional = ConditionalPackageDependency("os == 'linux'", ListOrItemPackageDependency(["package1"]), ListOrItemPackageDependency(["package3"]))
    item = ItemPackageDependency.new_from_conditional(conditional)

    item.conditional.condition = "jora"

    # this is a known issue with the current implementation
    assert item.conditional.condition == "os == 'linux'"
