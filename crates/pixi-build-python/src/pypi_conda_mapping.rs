use std::collections::HashMap;
use std::path::Path;

use miette::Diagnostic;
use once_cell::sync::Lazy;
use serde::{Deserialize, Serialize};
use tracing::{debug, info, warn};

/// Error types for PyPI to Conda mapping operations
#[derive(Debug, thiserror::Error, Diagnostic)]
pub enum PypiCondaMappingError {
    #[error("failed to read mapping file: {0}")]
    ReadFile(#[from] std::io::Error),

    #[error("failed to parse mapping YAML: {0}")]
    ParseYaml(#[from] serde_yaml::Error),

    #[error("PyPI package '{package}' has no conda mapping")]
    #[diagnostic(help("Add '{package}' to the mapping file or to your pixi.toml dependencies"))]
    UnmappedPackage { package: String },

    #[error("Multiple PyPI packages have no conda mapping: {packages}")]
    #[diagnostic(help("Add these packages to the mapping file or to your pixi.toml dependencies"))]
    MultipleUnmappedPackages { packages: String },
}

/// Represents the conda package mapping for a PyPI package
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CondaMapping {
    /// List of conda package names that correspond to this PyPI package
    /// Empty list means the package is built-in or not needed in conda
    pub conda: Vec<String>,
}

/// Type alias for the complete mapping from PyPI package names to conda packages
pub type PypiCondaMap = HashMap<String, CondaMapping>;

/// Global lazy-loaded mapping from the embedded YAML file
static BUILTIN_MAPPING: Lazy<PypiCondaMap> = Lazy::new(|| {
    let mapping_yaml = include_str!("../pypi-to-conda-mapping.yaml");
    serde_yaml::from_str(mapping_yaml)
        .expect("Built-in PyPI to conda mapping file should be valid YAML")
});

/// PyPI to Conda package mapper
#[derive(Debug, Clone)]
pub struct PypiCondaMapper {
    mapping: PypiCondaMap,
}

impl PypiCondaMapper {
    /// Create a new mapper with the built-in mapping
    pub fn new() -> Self {
        Self {
            mapping: BUILTIN_MAPPING.clone(),
        }
    }

    /// Create a new mapper from a custom mapping file
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, PypiCondaMappingError> {
        let content = fs_err::read_to_string(path)?;
        let mapping: PypiCondaMap = serde_yaml::from_str(&content)?;

        Ok(Self { mapping })
    }

    /// Create a new mapper by merging the built-in mapping with a custom file
    /// Custom mappings override built-in ones for the same package names
    pub fn with_custom_file<P: AsRef<Path>>(path: P) -> Result<Self, PypiCondaMappingError> {
        let mut mapping = BUILTIN_MAPPING.clone();

        let content = fs_err::read_to_string(path)?;
        let custom_mapping: PypiCondaMap = serde_yaml::from_str(&content)?;

        // Merge custom mappings, overriding built-in ones
        for (pypi_name, conda_mapping) in custom_mapping {
            mapping.insert(pypi_name, conda_mapping);
        }

        Ok(Self { mapping })
    }

    /// Map a single PyPI package name to conda package names
    /// Returns None if no mapping exists for the package
    pub fn map_package(&self, pypi_name: &str) -> Option<&[String]> {
        self.mapping.get(pypi_name).map(|m| m.conda.as_slice())
    }

    /// Map a PyPI package name to conda package names, falling back to the original name
    /// if no mapping exists
    pub fn map_package_or_fallback(&self, pypi_name: &str) -> Vec<String> {
        self.map_package_or_fallback_with_tracing(pypi_name, true)
    }

    /// Map a PyPI package name to conda package names, erroring if no mapping exists
    pub fn map_package_strict(
        &self,
        pypi_name: &str,
    ) -> Result<Vec<String>, PypiCondaMappingError> {
        match self.map_package(pypi_name) {
            Some(conda_names) => {
                if conda_names.is_empty() {
                    info!(
                        "PyPI package '{}' is built-in to Python, no conda package needed",
                        pypi_name
                    );
                    Ok(Vec::new())
                } else {
                    debug!(
                        "Mapped PyPI package '{}' to conda packages: {:?}",
                        pypi_name, conda_names
                    );
                    Ok(conda_names.to_vec())
                }
            }
            None => {
                warn!("No PyPI to conda mapping found for '{}'", pypi_name);
                Err(PypiCondaMappingError::UnmappedPackage {
                    package: pypi_name.to_string(),
                })
            }
        }
    }

    /// Internal method for mapping with optional tracing to avoid duplication in batch operations
    fn map_package_or_fallback_with_tracing(
        &self,
        pypi_name: &str,
        with_tracing: bool,
    ) -> Vec<String> {
        match self.map_package(pypi_name) {
            Some(conda_names) => {
                if conda_names.is_empty() {
                    // Empty mapping means built-in or not needed
                    if with_tracing {
                        info!(
                            "PyPI package '{}' is built-in to Python, no conda package needed",
                            pypi_name
                        );
                    }
                    Vec::new()
                } else {
                    if with_tracing {
                        debug!(
                            "Mapped PyPI package '{}' to conda packages: {:?}",
                            pypi_name, conda_names
                        );
                    }
                    conda_names.to_vec()
                }
            }
            None => {
                // No mapping found, use original name as fallback
                if with_tracing {
                    warn!(
                        "No PyPI to conda mapping found for '{}', using original name as fallback",
                        pypi_name
                    );
                }
                vec![pypi_name.to_string()]
            }
        }
    }

    /// Map multiple PyPI package names to conda package names
    /// Returns a Vec of tuples: (original_pypi_name, mapped_conda_names)
    pub fn map_packages<'a, I>(&self, pypi_names: I) -> Vec<(String, Vec<String>)>
    where
        I: IntoIterator<Item = &'a str>,
    {
        let pypi_names_vec: Vec<&str> = pypi_names.into_iter().collect();
        debug!(
            "Starting batch mapping for {} PyPI packages",
            pypi_names_vec.len()
        );

        let mut mapped_count = 0;
        let mut builtin_count = 0;
        let mut fallback_count = 0;

        let results: Vec<(String, Vec<String>)> = pypi_names_vec
            .into_iter()
            .map(|name| {
                let result = match self.map_package(name) {
                    Some(conda_names) => {
                        if conda_names.is_empty() {
                            builtin_count += 1;
                            info!("PyPI package '{}' is built-in to Python, no conda package needed", name);
                            Vec::new()
                        } else {
                            mapped_count += 1;
                            debug!("Mapped PyPI package '{}' to conda packages: {:?}", name, conda_names);
                            conda_names.to_vec()
                        }
                    }
                    None => {
                        fallback_count += 1;
                        warn!("No PyPI to conda mapping found for '{}', using original name as fallback", name);
                        vec![name.to_string()]
                    }
                };
                (name.to_string(), result)
            })
            .collect();

        debug!(
            "Batch mapping completed: {} mapped, {} built-in, {} fallbacks",
            mapped_count, builtin_count, fallback_count
        );

        results
    }

    /// Map multiple PyPI package names to conda package names in strict mode
    /// Returns an error if any packages are unmapped
    pub fn map_packages_strict<'a, I>(
        &self,
        pypi_names: I,
    ) -> Result<Vec<(String, Vec<String>)>, PypiCondaMappingError>
    where
        I: IntoIterator<Item = &'a str>,
    {
        let pypi_names_vec: Vec<&str> = pypi_names.into_iter().collect();
        debug!(
            "Starting strict batch mapping for {} PyPI packages",
            pypi_names_vec.len()
        );

        let mut mapped_count = 0;
        let mut builtin_count = 0;
        let mut unmapped_packages = Vec::new();
        let mut results = Vec::new();

        for name in &pypi_names_vec {
            match self.map_package(name) {
                Some(conda_names) => {
                    if conda_names.is_empty() {
                        builtin_count += 1;
                        info!(
                            "PyPI package '{}' is built-in to Python, no conda package needed",
                            name
                        );
                        results.push((name.to_string(), Vec::new()));
                    } else {
                        mapped_count += 1;
                        debug!(
                            "Mapped PyPI package '{}' to conda packages: {:?}",
                            name, conda_names
                        );
                        results.push((name.to_string(), conda_names.to_vec()));
                    }
                }
                None => {
                    warn!("No PyPI to conda mapping found for '{}'", name);
                    unmapped_packages.push(name.to_string());
                }
            }
        }

        if !unmapped_packages.is_empty() {
            if unmapped_packages.len() == 1 {
                return Err(PypiCondaMappingError::UnmappedPackage {
                    package: unmapped_packages[0].clone(),
                });
            } else {
                return Err(PypiCondaMappingError::MultipleUnmappedPackages {
                    packages: unmapped_packages.join(", "),
                });
            }
        }

        debug!(
            "Strict batch mapping completed successfully: {} mapped, {} built-in",
            mapped_count, builtin_count
        );

        Ok(results)
    }

    /// Check if a PyPI package has a mapping
    pub fn has_mapping(&self, pypi_name: &str) -> bool {
        self.mapping.contains_key(pypi_name)
    }

    /// Get all PyPI package names that have mappings
    pub fn get_mapped_packages(&self) -> Vec<&str> {
        self.mapping.keys().map(|s| s.as_str()).collect()
    }

    /// Get the total number of mappings
    pub fn mapping_count(&self) -> usize {
        self.mapping.len()
    }
}

impl Default for PypiCondaMapper {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;

    #[test]
    fn test_builtin_mapping_loads() {
        let mapper = PypiCondaMapper::new();
        assert!(mapper.mapping_count() > 0);
    }

    #[test]
    fn test_map_common_packages() {
        let mapper = PypiCondaMapper::new();

        // Test exact mapping
        assert_eq!(
            mapper.map_package("numpy"),
            Some(vec!["numpy".to_string()].as_slice())
        );
        assert_eq!(
            mapper.map_package("torch"),
            Some(vec!["pytorch".to_string()].as_slice())
        );

        // Test built-in package (empty mapping)
        assert_eq!(mapper.map_package("asyncio"), Some([].as_slice()));

        // Test non-existent package
        assert_eq!(mapper.map_package("nonexistent-package"), None);
    }

    #[test]
    fn test_map_package_or_fallback() {
        let mapper = PypiCondaMapper::new();

        // Test exact mapping
        assert_eq!(mapper.map_package_or_fallback("numpy"), vec!["numpy"]);

        // Test built-in package
        assert_eq!(
            mapper.map_package_or_fallback("asyncio"),
            Vec::<String>::new()
        );

        // Test fallback for unknown package
        assert_eq!(
            mapper.map_package_or_fallback("unknown-package"),
            vec!["unknown-package"]
        );
    }

    #[test]
    fn test_map_package_strict() {
        let mapper = PypiCondaMapper::new();

        // Test exact mapping
        assert_eq!(mapper.map_package_strict("numpy").unwrap(), vec!["numpy"]);

        // Test built-in package
        assert_eq!(
            mapper.map_package_strict("asyncio").unwrap(),
            Vec::<String>::new()
        );

        // Test error for unknown package
        let result = mapper.map_package_strict("unknown-package");
        assert!(result.is_err());
        match result.unwrap_err() {
            PypiCondaMappingError::UnmappedPackage { package } => {
                assert_eq!(package, "unknown-package");
            }
            _ => panic!("Expected UnmappedPackage error"),
        }
    }

    #[test]
    fn test_map_packages_strict() {
        let mapper = PypiCondaMapper::new();

        // Test successful strict mapping
        let packages = vec!["numpy", "asyncio"];
        let result = mapper
            .map_packages_strict(packages.iter().copied())
            .unwrap();
        assert_eq!(result.len(), 2);
        assert_eq!(result[0], ("numpy".to_string(), vec!["numpy".to_string()]));
        assert_eq!(result[1], ("asyncio".to_string(), Vec::<String>::new()));

        // Test error for single unknown package
        let packages = vec!["numpy", "unknown-package"];
        let result = mapper.map_packages_strict(packages.iter().copied());
        assert!(result.is_err());
        match result.unwrap_err() {
            PypiCondaMappingError::UnmappedPackage { package } => {
                assert_eq!(package, "unknown-package");
            }
            _ => panic!("Expected UnmappedPackage error"),
        }

        // Test error for multiple unknown packages
        let packages = vec!["unknown1", "numpy", "unknown2"];
        let result = mapper.map_packages_strict(packages.iter().copied());
        assert!(result.is_err());
        match result.unwrap_err() {
            PypiCondaMappingError::MultipleUnmappedPackages { packages } => {
                assert!(packages.contains("unknown1"));
                assert!(packages.contains("unknown2"));
                assert!(!packages.contains("numpy"));
            }
            _ => panic!("Expected MultipleUnmappedPackages error"),
        }
    }

    #[test]
    fn test_map_multiple_packages() {
        let mapper = PypiCondaMapper::new();
        let packages = vec!["numpy", "torch", "unknown-package"];

        let mapped = mapper.map_packages(packages.iter().copied());

        assert_eq!(mapped.len(), 3);
        assert_eq!(mapped[0], ("numpy".to_string(), vec!["numpy".to_string()]));
        assert_eq!(
            mapped[1],
            ("torch".to_string(), vec!["pytorch".to_string()])
        );
        assert_eq!(
            mapped[2],
            (
                "unknown-package".to_string(),
                vec!["unknown-package".to_string()]
            )
        );
    }

    #[test]
    fn test_custom_mapping_file() -> Result<(), Box<dyn std::error::Error>> {
        let mut temp_file = NamedTempFile::new()?;
        writeln!(
            temp_file,
            "custom-package:\n  conda: [custom-conda-package]"
        )?;

        let mapper = PypiCondaMapper::from_file(temp_file.path())?;

        assert_eq!(
            mapper.map_package("custom-package"),
            Some(vec!["custom-conda-package".to_string()].as_slice())
        );

        Ok(())
    }

    #[test]
    fn test_merge_with_custom_mapping() -> Result<(), Box<dyn std::error::Error>> {
        let mut temp_file = NamedTempFile::new()?;
        writeln!(
            temp_file,
            r#"
numpy:
  conda: [custom-numpy]
custom-package:
  conda: [custom-conda-package]
"#
        )?;

        let mapper = PypiCondaMapper::with_custom_file(temp_file.path())?;

        // Custom mapping should override built-in
        assert_eq!(
            mapper.map_package("numpy"),
            Some(vec!["custom-numpy".to_string()].as_slice())
        );

        // Custom package should be available
        assert_eq!(
            mapper.map_package("custom-package"),
            Some(vec!["custom-conda-package".to_string()].as_slice())
        );

        // Built-in packages not overridden should still work
        assert_eq!(
            mapper.map_package("torch"),
            Some(vec!["pytorch".to_string()].as_slice())
        );

        Ok(())
    }

    #[test]
    fn test_has_mapping() {
        let mapper = PypiCondaMapper::new();

        assert!(mapper.has_mapping("numpy"));
        assert!(mapper.has_mapping("asyncio")); // Even built-in packages have mappings
        assert!(!mapper.has_mapping("nonexistent-package"));
    }
}
