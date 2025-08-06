use super::config::{MojoBinConfig, MojoPkgConfig};
use minijinja::Environment;
use serde::Serialize;

#[derive(Debug, Serialize)]
pub struct BuildScriptContext {
    /// The directory where the source code is located, the manifest root.
    pub source_dir: String,
    /// Any executable artifacts to create.
    pub bins: Option<Vec<MojoBinConfig>>,
    /// Any packages to create.
    pub pkg: Option<MojoPkgConfig>,
}

impl BuildScriptContext {
    pub fn render(&self) -> Vec<String> {
        let env = Environment::new();
        let template = env
            .template_from_str(include_str!("build_script.j2"))
            .unwrap();
        let rendered = template.render(self).unwrap().to_string();
        rendered.lines().map(|s| s.to_string()).collect()
    }
}
