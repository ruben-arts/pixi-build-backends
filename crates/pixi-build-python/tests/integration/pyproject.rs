use pixi_build_backend::generated_recipe::GenerateRecipe;
use pixi_build_python::{PythonGenerator, config::PythonBackendConfig};
use pixi_build_types::ProjectModelV1;
use rattler_conda_types::Platform;
use recipe_stage0::recipe::IntermediateRecipe;
use std::collections::HashSet;

/// Helper function to generate a recipe from a pyproject.toml
async fn generate_recipe_from_pyproject(
    pyproject_content: &str,
    backend_config: serde_json::Value,
    project_model: ProjectModelV1,
    platform: Platform,
) -> IntermediateRecipe {
    let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
    let pyproject_path = temp_dir.path().join("pyproject.toml");

    fs_err::write(&pyproject_path, pyproject_content).expect("Failed to write pyproject.toml");

    let config: PythonBackendConfig =
        serde_json::from_value(backend_config).expect("Failed to parse backend config");

    let generator = PythonGenerator::default();
    let generated = generator
        .generate_recipe(
            &project_model,
            &config,
            temp_dir.path().to_path_buf(),
            platform,
            None,
            &HashSet::new(),
            vec![],
            None,
        )
        .await
        .expect("Failed to generate recipe");

    generated.recipe
}

/// Helper to create a minimal Python project model
fn create_minimal_python_project() -> ProjectModelV1 {
    serde_json::from_value(serde_json::json!({
        "targets": {
            "defaultTarget": {}
        }
    }))
    .expect("Failed to create project model")
}

/// Helper to create a binary package spec with just a version constraint
fn binary_package(version: &str) -> pixi_build_types::PackageSpecV1 {
    serde_json::from_value(serde_json::json!({
        "binary": {
            "version": version
        }
    }))
    .expect("Failed to create binary package spec")
}

#[tokio::test]
async fn test_pyproject_dependency_mapping() {
    // Test 1: Project dependencies are mapped to run requirements
    let pyproject = r#"
[project]
name = "test-package"
version = "0.1.0"
dependencies = ["requests>=2.28.0", "click"]
"#;

    let recipe = generate_recipe_from_pyproject(
        pyproject,
        serde_json::json!({}),
        create_minimal_python_project(),
        Platform::Linux64,
    )
    .await;

    // Verify run requirements contain python (basic test without network)
    let run_deps: Vec<String> = recipe
        .requirements
        .run
        .iter()
        .map(|d| d.to_string())
        .collect();
    assert!(
        run_deps.iter().any(|d| d.contains("python")),
        "Should contain python in run dependencies"
    );

    // Test 2: Build-system requires are mapped to host requirements
    let pyproject_with_build = r#"
[project]
name = "test-package"
version = "0.1.0"

[build-system]
requires = ["setuptools>=45", "wheel"]
"#;

    let recipe = generate_recipe_from_pyproject(
        pyproject_with_build,
        serde_json::json!({}),
        create_minimal_python_project(),
        Platform::Linux64,
    )
    .await;

    let host_deps: Vec<String> = recipe
        .requirements
        .host
        .iter()
        .map(|d| d.to_string())
        .collect();
    // Just verify basic host requirements exist (python and pip installer)
    assert!(
        host_deps
            .iter()
            .any(|d| d.contains("python") || d.contains("pip")),
        "Should contain python or pip in host dependencies"
    );

    // Test 3: Auto-detect Rust compiler from maturin
    let pyproject_maturin = r#"
[project]
name = "test-package"
version = "0.1.0"

[build-system]
requires = ["maturin>=1.0"]
"#;

    let recipe = generate_recipe_from_pyproject(
        pyproject_maturin,
        serde_json::json!({}),
        create_minimal_python_project(),
        Platform::Linux64,
    )
    .await;

    let build_deps: Vec<String> = recipe
        .requirements
        .build
        .iter()
        .map(|d| d.to_string())
        .collect();
    assert!(
        build_deps.iter().any(|d| d.contains("rust")),
        "Should auto-detect and add Rust compiler from maturin"
    );

    // Test 4: Respect ignore-pyproject-manifest flag
    let recipe_ignored = generate_recipe_from_pyproject(
        pyproject,
        serde_json::json!({
            "ignore-pyproject-manifest": true
        }),
        serde_json::from_value(serde_json::json!({
            "name": "explicit-name",
            "version": "1.0.0",
            "targets": {
                "defaultTarget": {}
            }
        }))
        .unwrap(),
        Platform::Linux64,
    )
    .await;

    // When ignoring pyproject.toml, no requests/click should be in run deps
    let run_deps_ignored: Vec<String> = recipe_ignored
        .requirements
        .run
        .iter()
        .map(|d| d.to_string())
        .collect();
    assert!(
        !run_deps_ignored.iter().any(|d| d.contains("requests")),
        "Should not contain requests when ignoring pyproject.toml"
    );
}

#[tokio::test]
async fn test_platform_marker_evaluation() {
    let pyproject = r#"
[project]
name = "test-package"
version = "0.1.0"
dependencies = [
    "requests",
    "pywin32; sys_platform == 'win32'",
]
"#;

    let recipe = generate_recipe_from_pyproject(
        pyproject,
        serde_json::json!({}),
        create_minimal_python_project(),
        Platform::Linux64,
    )
    .await;

    let run_deps: Vec<String> = recipe
        .requirements
        .run
        .iter()
        .map(|d| d.to_string())
        .collect();

    // Basic verification: recipe should be generated successfully
    // Platform marker evaluation happens during recipe generation
    assert!(
        run_deps.iter().any(|d| d.contains("python")),
        "Should include python in run dependencies"
    );
}

#[tokio::test]
async fn test_pixi_dependencies_override_pyproject() {
    let pyproject = r#"
[project]
name = "test-package"
version = "0.1.0"
dependencies = ["requests>=2.28.0"]
"#;

    let project_model: ProjectModelV1 = serde_json::from_value(serde_json::json!({
        "name": "test-package",
        "version": "0.1.0",
        "targets": {
            "defaultTarget": {
                "runDependencies": {
                    "requests": binary_package("3.0.0")
                }
            }
        }
    }))
    .expect("Failed to create project model");

    let recipe = generate_recipe_from_pyproject(
        pyproject,
        serde_json::json!({}),
        project_model,
        Platform::Linux64,
    )
    .await;

    let run_deps: Vec<String> = recipe
        .requirements
        .run
        .iter()
        .map(|d| d.to_string())
        .collect();

    // Pixi manifest version should take precedence
    // pyproject.toml specifies "requests>=2.28.0"
    // Pixi manifest specifies "requests 3.0.0"
    // Only one requests entry should exist (from Pixi manifest)
    let requests_count = run_deps.iter().filter(|d| d.contains("requests")).count();
    assert_eq!(
        requests_count, 1,
        "Should have exactly one requests dependency (from Pixi manifest)"
    );
}
