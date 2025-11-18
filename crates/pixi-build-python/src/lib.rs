//! PyPI to Conda package mapping library for pixi-build-python
//!
//! This library provides functionality to map PyPI package names to their
//! conda-forge equivalents, which is useful for converting Python dependencies
//! from pyproject.toml files to conda package specifications.

pub mod pypi_conda_mapping;

// Re-export the main mapper and error types for convenience
pub use pypi_conda_mapping::{PypiCondaMapper, PypiCondaMappingError};
