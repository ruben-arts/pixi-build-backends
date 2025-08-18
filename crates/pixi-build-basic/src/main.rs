mod config;

use std::{collections::BTreeSet, path::Path, sync::Arc};

use config::BasicBackendConfig;
use pixi_build_backend::{
    generated_recipe::{DefaultMetadataProvider, GenerateRecipe, GeneratedRecipe, PythonParams},
    intermediate_backend::IntermediateBackendInstantiator,
};
use recipe_stage0::recipe::Script;

#[derive(Default, Clone)]
pub struct BasicGenerator {}

impl GenerateRecipe for BasicGenerator {
    type Config = BasicBackendConfig;

    fn generate_recipe(
        &self,
        model: &pixi_build_types::ProjectModelV1,
        config: &Self::Config,
        _manifest_root: std::path::PathBuf,
        _host_platform: rattler_conda_types::Platform,
        _python_params: Option<PythonParams>,
    ) -> miette::Result<GeneratedRecipe> {
        let mut generated_recipe =
            GeneratedRecipe::from_model(model.clone(), &mut DefaultMetadataProvider)?;

        generated_recipe.recipe.build.script = Script {
            content: config.script.clone(),
            env: config.env.clone(),
            interpreter: config.interpreter.clone(),
            ..Default::default()
        };

        Ok(generated_recipe)
    }

    fn extract_input_globs_from_build(
        config: &Self::Config,
        _workdir: impl AsRef<Path>,
        _editable: bool,
    ) -> BTreeSet<String> {
        [].iter()
            .map(|s: &&str| s.to_string())
            .chain(config.extra_input_globs.clone())
            .collect()
    }
}

#[tokio::main]
pub async fn main() {
    if let Err(err) = pixi_build_backend::cli::main(|log| {
        IntermediateBackendInstantiator::<BasicGenerator>::new(log, Arc::default())
    })
    .await
    {
        eprintln!("{err:?}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use pixi_build_types::ProjectModelV1;
    use serde_json::json;
    use std::path::PathBuf;
    use std::str::FromStr;

    use super::*;

    #[test]
    fn test_input_globs_includes_extra_globs() {
        let config = BasicBackendConfig {
            extra_input_globs: vec!["custom/*.c".to_string()],
            ..Default::default()
        };

        let result = BasicGenerator::extract_input_globs_from_build(&config, PathBuf::new(), false);

        insta::assert_debug_snapshot!(result);
    }

    #[test]
    fn test_interpreter_is_set_in_script() {
        let model = ProjectModelV1 {
            name: "test".to_string(),
            version: Some(rattler_conda_types::Version::from_str("1.0.0").unwrap()),
            ..Default::default()
        };

        let config = BasicBackendConfig {
            script: vec!["echo 'hello'".to_string(), "ls -la".to_string()],
            interpreter: Some("bash".to_string()),
            ..Default::default()
        };

        let generator = BasicGenerator::default();
        let result = generator
            .generate_recipe(
                &model,
                &config,
                PathBuf::new(),
                rattler_conda_types::Platform::Linux64,
                None,
            )
            .unwrap();

        assert_eq!(
            result.recipe.build.script.interpreter,
            Some("bash".to_string())
        );
        assert_eq!(result.recipe.build.script.content, config.script);
    }

    #[test]
    fn test_no_interpreter_keeps_original_commands() {
        let model = ProjectModelV1 {
            name: "test".to_string(),
            version: Some(rattler_conda_types::Version::from_str("1.0.0").unwrap()),
            ..Default::default()
        };

        let config = BasicBackendConfig {
            script: vec!["echo 'hello'".to_string(), "ls -la".to_string()],
            interpreter: None,
            ..Default::default()
        };

        let generator = BasicGenerator::default();
        let result = generator
            .generate_recipe(
                &model,
                &config,
                PathBuf::new(),
                rattler_conda_types::Platform::Linux64,
                None,
            )
            .unwrap();

        let expected_content = vec!["echo 'hello'".to_string(), "ls -la".to_string()];

        assert_eq!(result.recipe.build.script.content, expected_content);
        assert_eq!(result.recipe.build.script.interpreter, None);
    }

    #[test]
    fn test_interpreter_from_project_model_config() {
        let model = ProjectModelV1 {
            name: "test".to_string(),
            version: Some(rattler_conda_types::Version::from_str("1.0.0").unwrap()),
            ..Default::default()
        };

        // Create a config with interpreter setting
        let config_json = json!({
            "script": ["echo 'hello from python'", "print('test')"],
            "interpreter": "python"
        });

        let config: BasicBackendConfig = serde_json::from_value(config_json).unwrap();

        let generator = BasicGenerator::default();
        let result = generator
            .generate_recipe(
                &model,
                &config,
                PathBuf::new(),
                rattler_conda_types::Platform::Linux64,
                None,
            )
            .unwrap();

        assert_eq!(
            result.recipe.build.script.interpreter,
            Some("python".to_string())
        );
        assert_eq!(
            result.recipe.build.script.content,
            vec![
                "echo 'hello from python'".to_string(),
                "print('test')".to_string()
            ]
        );
    }
}
