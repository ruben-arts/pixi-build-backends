mod config;

use std::{collections::BTreeSet, path::Path, sync::Arc};

use config::BasicBackendConfig;
use miette::IntoDiagnostic;
use pixi_build_backend::{
    generated_recipe::{GenerateRecipe, GeneratedRecipe, PythonParams},
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
        manifest_root: std::path::PathBuf,
        host_platform: rattler_conda_types::Platform,
        _python_params: Option<PythonParams>,
    ) -> miette::Result<GeneratedRecipe> {
        let mut generated_recipe = GeneratedRecipe::from_model(model.clone());

        generated_recipe.recipe.build.script = Script {
            content: config.script.clone(),
            env: config.env.clone(),
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
    use std::path::PathBuf;

    use indexmap::IndexMap;
    use pixi_build_backend::protocol::ProtocolInstantiator;
    use pixi_build_types::{
        ProjectModelV1,
        procedures::{conda_outputs::CondaOutputsParams, initialize::InitializeParams},
    };
    use rattler_build::console_utils::LoggingOutputHandler;

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
}
