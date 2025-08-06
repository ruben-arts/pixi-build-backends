mod build_script;
mod config;

use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
};

use build_script::BuildScriptContext;
use cargo_toml::{Error as CargoTomlError, Manifest};
use config::RustBackendConfig;
use miette::IntoDiagnostic;
use pixi_build_backend::generated_recipe::{MetadataProvider, MetadataProviderError};
use pixi_build_backend::{
    cache::{sccache_envs, sccache_tools},
    compilers::{Language, compiler_requirement},
    generated_recipe::{GenerateRecipe, GeneratedRecipe, PythonParams},
    intermediate_backend::IntermediateBackendInstantiator,
};
use pixi_build_types::ProjectModelV1;
use rattler_conda_types::{PackageName, Platform, Version};
use recipe_stage0::recipe::{About, Value};
use recipe_stage0::{
    matchspec::PackageDependency,
    recipe::{Item, Script},
};
use std::collections::BTreeSet;
use std::str::FromStr;

#[derive(Default, Clone)]
pub struct RustGenerator {}

pub struct CargoMetadataProvider {
    cargo_manifest: Manifest,
}
impl MetadataProvider for CargoMetadataProvider {
    fn name(&self) -> Result<String, MetadataProviderError> {
        if let Some(package) = &self.cargo_manifest.package {
            Ok(package.name.clone().to_string())
        } else {
            Err(MetadataProviderError::CannotProvideName)
        }
    }
    fn version(&self) -> Result<Version, MetadataProviderError> {
        if let Some(package) = &self.cargo_manifest.package {
            if let Ok(version) = &package.version.get() {
                Version::from_str(version).map_err(|e| MetadataProviderError::CannotParseVersion(e))
            } else {
                Err(MetadataProviderError::CannotProvideVersion)
            }
        } else {
            Err(MetadataProviderError::CannotProvideVersion)
        }
    }

    fn about(&self) -> Option<About> {
        // Macro to simplify setting str values
        macro_rules! set_str {
            ($target:expr, $source:expr) => {
                if $target.is_none() {
                    if let Some(val) = &$source {
                        if let Ok(val) = val.get() {
                            $target = Some(Value::from_str(val.as_str()).unwrap());
                        }
                    }
                }
            };
        }

        // Translate the Cargo manifest into an recipe About section
        if let Some(cargo_package) = &self.cargo_manifest.package {
            let mut about = About::default();
            set_str!(about.description, cargo_package.description);
            set_str!(about.documentation, cargo_package.documentation);
            set_str!(about.repository, cargo_package.repository);
            set_str!(about.license, cargo_package.license);
            set_str!(about.homepage, cargo_package.homepage);

            if about.license_file.is_none() {
                // If license file is not set, use the first license file from the package
                if let Some(license_file) = &cargo_package.license_file {
                    if let Ok(license_file) = license_file.get() {
                        about.license_file = Some(
                            Value::from_str(license_file.to_string_lossy().to_string().as_str())
                                .unwrap(),
                        );
                    }
                }
            }

            // If summary is not set, use the package description as a fallback.
            if about.summary.is_none() {
                if let Some(description) = &cargo_package.description {
                    if let Ok(description) = description.get() {
                        about.summary = Some(Value::from_str(description.as_str()).unwrap());
                    }
                }
            }
            return Some(about);
        }
        None
    }
}
impl GenerateRecipe for RustGenerator {
    type Config = RustBackendConfig;

    fn generate_recipe(
        &self,
        model: &ProjectModelV1,
        config: &Self::Config,
        manifest_root: PathBuf,
        host_platform: Platform,
        _python_params: Option<PythonParams>,
    ) -> miette::Result<GeneratedRecipe> {
        let provider = if config.ignore_cargo_manifest.is_some_and(|ignore| ignore) {
            None
        } else {
            Some(CargoMetadataProvider {
                cargo_manifest: get_cargo_manifest(&manifest_root).into_diagnostic()?,
            })
        };

        let mut generated_recipe =
            GeneratedRecipe::from_model(model.clone(), provider).into_diagnostic()?;

        // we need to add compilers
        let compiler_function = compiler_requirement(&Language::Rust);

        let requirements = &mut generated_recipe.recipe.requirements;

        let resolved_requirements = requirements.resolve(Some(host_platform));

        // Mix the Cargo manifest with the recipe

        // Ensure the compiler function is added to the build requirements
        // only if it is not already present.

        if !resolved_requirements
            .build
            .contains_key(&PackageName::new_unchecked(Language::Rust.to_string()))
        {
            requirements.build.push(compiler_function.clone());
        }

        let has_openssl = resolved_requirements.contains(&"openssl".parse().into_diagnostic()?);

        let mut has_sccache = false;

        let config_env = config.env.clone();

        let system_env_vars = std::env::vars().collect::<HashMap<String, String>>();

        let all_env_vars = config_env
            .clone()
            .into_iter()
            .chain(system_env_vars.clone())
            .collect();

        let mut sccache_secrets = Vec::default();

        // Verify if user has set any sccache environment variables
        if sccache_envs(&all_env_vars).is_some() {
            // check if we set some sccache in system env vars
            if let Some(system_sccache_keys) = sccache_envs(&system_env_vars) {
                // If sccache_envs are used in the system environment variables,
                // we need to set them as secrets
                let system_sccache_keys = system_env_vars
                    .keys()
                    // we set only those keys that are present in the system environment variables and not in the config env
                    .filter(|key| {
                        system_sccache_keys.contains(&key.as_str())
                            && !config_env.contains_key(*key)
                    })
                    .cloned()
                    .collect();

                sccache_secrets = system_sccache_keys;
            };

            let sccache_dep: Vec<Item<PackageDependency>> = sccache_tools()
                .iter()
                .map(|tool| tool.parse().into_diagnostic())
                .collect::<miette::Result<Vec<_>>>()?;

            // Add sccache tools to the build requirements
            // only if they are not already present
            let existing_reqs: Vec<_> = requirements.build.clone().into_iter().collect();

            has_sccache = true;
        }

        let build_script = BuildScriptContext {
            source_dir: manifest_root.display().to_string(),
            extra_args: config.extra_args.clone(),
            has_openssl,
            has_sccache,
            is_bash: !Platform::current().is_windows(),
        }
        .render();

        generated_recipe.recipe.build.script = Script {
            content: build_script,
            env: config_env,
            secrets: sccache_secrets,
        };

        Ok(generated_recipe)
    }

    /// Returns the build input globs used by the backend.
    fn extract_input_globs_from_build(
        config: &Self::Config,
        _workdir: impl AsRef<Path>,
        _editable: bool,
    ) -> BTreeSet<String> {
        [
            "**/*.rs",
            // Cargo configuration files
            "Cargo.toml",
            "Cargo.lock",
            // Build scripts
            "build.rs",
        ]
        .iter()
        .map(|s| s.to_string())
        .chain(config.extra_input_globs.clone())
        .collect()
    }
}

fn get_cargo_manifest(manifest_root: &PathBuf) -> Result<Manifest, CargoTomlError> {
    let package_manifest_path = manifest_root.join("Cargo.toml");

    Manifest::from_path(&package_manifest_path).and_then(|mut manifest| {
        manifest.complete_from_path(&package_manifest_path)?;
        Ok(manifest)
    })
}

#[tokio::main]
pub async fn main() {
    if let Err(err) = pixi_build_backend::cli::main(|log| {
        IntermediateBackendInstantiator::<RustGenerator>::new(log, Arc::default())
    })
    .await
    {
        eprintln!("{err:?}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use indexmap::IndexMap;

    use super::*;

    #[test]
    fn test_input_globs_includes_extra_globs() {
        let config = RustBackendConfig {
            extra_input_globs: vec!["custom/*.txt".to_string(), "extra/**/*.py".to_string()],
            ..Default::default()
        };

        let result = RustGenerator::extract_input_globs_from_build(&config, PathBuf::new(), false);

        // Verify that all extra globs are included in the result
        for extra_glob in &config.extra_input_globs {
            assert!(
                result.contains(extra_glob),
                "Result should contain extra glob: {}",
                extra_glob
            );
        }

        // Verify that default globs are still present
        assert!(result.contains("**/*.rs"));
        assert!(result.contains("Cargo.toml"));
        assert!(result.contains("Cargo.lock"));
        assert!(result.contains("build.rs"));
    }

    #[macro_export]
    macro_rules! project_fixture {
        ($($json:tt)+) => {
            serde_json::from_value::<ProjectModelV1>(
                serde_json::json!($($json)+)
            ).expect("Failed to create TestProjectModel from JSON fixture.")
        };
    }

    #[test]
    fn test_rust_is_in_build_requirements() {
        let project_model = project_fixture!({
            "name": "foobar",
            "version": "0.1.0",
            "targets": {
                "defaultTarget": {
                    "runDependencies": {
                        "boltons": {
                            "binary": {
                                "version": "*"
                            }
                        }
                    }
                },
            }
        });

        let generated_recipe = RustGenerator::default()
            .generate_recipe(
                &project_model,
                &RustBackendConfig::default_with_ignore_cargo_manifest(),
                PathBuf::from("."),
                Platform::Linux64,
                None,
            )
            .expect("Failed to generate recipe");

        insta::assert_yaml_snapshot!(generated_recipe.recipe, {
        ".source[0].path" => "[ ... path ... ]",
        ".build.script" => "[ ... script ... ]",
        });
    }

    #[test]
    fn test_rust_is_not_added_if_already_present() {
        let project_model = project_fixture!({
            "name": "foobar",
            "version": "0.1.0",
            "targets": {
                "defaultTarget": {
                    "runDependencies": {
                        "boltons": {
                            "binary": {
                                "version": "*"
                            }
                        }
                    },
                    "buildDependencies": {
                        "rust": {
                            "binary": {
                                "version": "*"
                            }
                        }
                    }
                },
            }
        });

        let generated_recipe = RustGenerator::default()
            .generate_recipe(
                &project_model,
                &RustBackendConfig::default_with_ignore_cargo_manifest(),
                PathBuf::from("."),
                Platform::Linux64,
                None,
            )
            .expect("Failed to generate recipe");

        insta::assert_yaml_snapshot!(generated_recipe.recipe, {
        ".source[0].path" => "[ ... path ... ]",
        ".build.script" => "[ ... script ... ]",
        });
    }

    #[test]
    fn test_env_vars_are_set() {
        let project_model = project_fixture!({
            "name": "foobar",
            "version": "0.1.0",
            "targets": {
                "defaultTarget": {
                    "runDependencies": {
                        "boltons": {
                            "binary": {
                                "version": "*"
                            }
                        }
                    }
                },
            }
        });

        let env = IndexMap::from([("foo".to_string(), "bar".to_string())]);

        let generated_recipe = RustGenerator::default()
            .generate_recipe(
                &project_model,
                &RustBackendConfig {
                    env: env.clone(),
                    ignore_cargo_manifest: Some(true),
                    ..Default::default()
                },
                PathBuf::from("."),
                Platform::Linux64,
                None,
            )
            .expect("Failed to generate recipe");

        insta::assert_yaml_snapshot!(generated_recipe.recipe.build.script,
        {
            ".content" => "[ ... script ... ]",
        });
    }

    #[test]
    fn test_sccache_is_enabled() {
        let project_model = project_fixture!({
            "name": "foobar",
            "version": "0.1.0",
            "targets": {
                "default_target": {
                    "run_dependencies": {
                        "boltons": "*"
                    }
                },
            }
        });

        let env = IndexMap::from([("SCCACHE_BUCKET".to_string(), "my-bucket".to_string())]);

        let system_env_vars = [
            ("SCCACHE_SYSTEM", Some("SOME_VALUE")),
            // We want to test that config env variable wins over system env variable
            ("SCCACHE_BUCKET", Some("system-bucket")),
        ];

        let generated_recipe = temp_env::with_vars(system_env_vars, || {
            RustGenerator::default()
                .generate_recipe(
                    &project_model,
                    &RustBackendConfig {
                        env,
                        ignore_cargo_manifest: Some(true),
                        ..Default::default()
                    },
                    PathBuf::from("."),
                    Platform::Linux64,
                    None,
                )
                .expect("Failed to generate recipe")
        });

        // Verify that sccache is added to the build requirements
        // when some env variables are set
        insta::assert_yaml_snapshot!(generated_recipe.recipe, {
        ".source[0].path" => "[ ... path ... ]",
        ".build.script.content" => "[ ... script ... ]",
        });
    }
    #[test]
    fn test_with_cargo_manifest() {
        let project_model = project_fixture!({
            "name": "",
            "targets": {
                "default_target": {
                    "run_dependencies": {
                        "dependency": "*"
                    }
                },
            }
        });

        let generated_recipe = RustGenerator::default()
            .generate_recipe(
                &project_model,
                &RustBackendConfig::default(),
                // Using this crate itself, as it has interesting metadata, using .workspace
                std::env::current_dir().unwrap(),
                Platform::Linux64,
                None,
            )
            .expect("Failed to generate recipe");

        // Verify that the about section is populated correctly
        eprintln!("{:#?}", generated_recipe.recipe);

        // Manually load the Cargo manifest to ensure it works
        let current_dir = std::env::current_dir().unwrap();
        let package_manifest_path = current_dir.join("Cargo.toml");
        let mut manifest = Manifest::from_path(&package_manifest_path).unwrap();
        manifest.complete_from_path(&package_manifest_path).unwrap();

        assert_eq!(
            manifest.clone().package.unwrap().name.clone(),
            generated_recipe.recipe.package.name.to_string()
        );
        assert_eq!(
            *manifest.clone().package.unwrap().version.get().unwrap(),
            generated_recipe.recipe.package.version.to_string()
        );
        assert_eq!(
            *manifest
                .clone()
                .package
                .unwrap()
                .description
                .unwrap()
                .get()
                .unwrap(),
            generated_recipe
                .recipe
                .about
                .as_ref()
                .and_then(|a| a.description.clone())
                .unwrap()
                .to_string()
        );
        assert_eq!(
            *manifest
                .clone()
                .package
                .unwrap()
                .license
                .unwrap()
                .get()
                .unwrap(),
            generated_recipe
                .recipe
                .about
                .as_ref()
                .and_then(|a| a.license.clone())
                .unwrap()
                .to_string()
        );
        assert_eq!(
            *manifest
                .clone()
                .package
                .unwrap()
                .repository
                .unwrap()
                .get()
                .unwrap(),
            generated_recipe
                .recipe
                .about
                .as_ref()
                .and_then(|a| a.repository.clone())
                .unwrap()
                .to_string()
        );
    }
}
