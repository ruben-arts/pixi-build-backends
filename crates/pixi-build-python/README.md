# PyPI to Conda Package Mapping for pixi-build-python

This crate provides functionality to map PyPI package names to their conda-forge equivalents, which is essential for converting Python dependencies from `pyproject.toml` files to conda package specifications.

## Features

- **Built-in mapping**: Comprehensive mapping of 90+ common PyPI packages to conda-forge names
- **Custom mappings**: Support for user-defined mapping files
- **Fallback behavior**: Graceful handling of unmapped packages
- **Batch processing**: Efficient mapping of multiple packages at once
- **Flexible API**: Multiple ways to query and use mappings

## Quick Start

```rust
use pixi_build_python::PypiCondaMapper;

// Create a mapper with built-in mappings
let mapper = PypiCondaMapper::new();

// Map a single package
let conda_packages = mapper.map_package("torch");
assert_eq!(conda_packages, Some(["pytorch"].as_slice()));

// Map with fallback (returns original name if no mapping exists)
let conda_packages = mapper.map_package_or_fallback("unknown-package");
assert_eq!(conda_packages, vec!["unknown-package"]);

// Batch mapping
let pypi_packages = vec!["numpy", "torch", "scikit-learn"];
let mapped = mapper.map_packages(pypi_packages.iter().copied());
// Returns: [("numpy", ["numpy"]), ("torch", ["pytorch"]), ("scikit-learn", ["scikit-learn"])]
```

## Tracing and Logging

The mapping system provides comprehensive logging through the `tracing` crate:

```rust
use tracing_subscriber;
use tracing::Level;

// Initialize tracing to see mapping behavior
tracing_subscriber::fmt()
    .with_max_level(Level::DEBUG)
    .init();

let mapper = PypiCondaMapper::new();

// This will log different levels based on the mapping result:
mapper.map_package_or_fallback("numpy");        // DEBUG: successful mapping
mapper.map_package_or_fallback("asyncio");      // INFO: built-in package  
mapper.map_package_or_fallback("unknown-pkg");  // WARN: fallback used
```

### Log Levels

- **DEBUG**: Successful mappings (`numpy -> ["numpy"]`)
- **INFO**: Built-in packages that don't need conda equivalents (`asyncio`)
- **WARN**: Fallback cases where no mapping exists (`unknown-pkg -> ["unknown-pkg"]`)
- **DEBUG**: Batch operation summaries (`Batch mapping completed: 4 mapped, 1 built-in, 2 fallbacks`)

## Strict vs Fallback Mode

The mapper supports two modes for handling unmapped packages:

### Fallback Mode (Default)
```rust
// Returns original name if no mapping exists
let conda_packages = mapper.map_package_or_fallback("unknown-pkg");
// Result: ["unknown-pkg"]
```

### Strict Mode (Recommended)
```rust
// Errors on unmapped packages with helpful guidance
match mapper.map_package_strict("unknown-pkg") {
    Ok(conda_packages) => println!("Mapped: {:?}", conda_packages),
    Err(e) => eprintln!("Error: {}", e),
    // Error: PyPI package 'unknown-pkg' has no conda mapping
    // Help: Add 'unknown-pkg' to the mapping file or to your pixi.toml dependencies
}
```

## Built-in Mappings

The crate includes mappings for common Python packages across various domains:

### Scientific Computing
- `numpy` → `numpy`
- `scipy` → `scipy`
- `pandas` → `pandas`
- `matplotlib` → `matplotlib`

### Machine Learning
- `torch` → `pytorch`
- `tensorflow` → `tensorflow`
- `scikit-learn` → `scikit-learn`
- `xgboost` → `xgboost`

### Web Frameworks
- `django` → `django`
- `flask` → `flask`
- `fastapi` → `fastapi`
- `requests` → `requests`

### Image Processing
- `opencv-python` → `opencv`
- `pillow` → `pillow`

And many more! See `pypi-to-conda-mapping.yaml` for the complete list.

## Special Cases

### Built-in Packages
Some PyPI packages are built into Python and don't need conda equivalents:
```rust
let mapper = PypiCondaMapper::new();
assert_eq!(mapper.map_package("asyncio"), Some([].as_slice())); // Empty = built-in
```

### Name Differences
Some packages have different names between PyPI and conda-forge:
- `torch` (PyPI) → `pytorch` (conda)
- `opencv-python` (PyPI) → `opencv` (conda)
- `msgpack` (PyPI) → `msgpack-python` (conda)

## Custom Mappings

You can extend or override the built-in mappings:

### From File
```rust
// Load custom mappings from a YAML file
let mapper = PypiCondaMapper::from_file("my-mappings.yaml")?;

// Or merge with built-in mappings (custom overrides built-in)
let mapper = PypiCondaMapper::with_custom_file("my-mappings.yaml")?;
```

### Custom Mapping File Format
```yaml
# my-mappings.yaml
my-custom-package:
  conda: [my-conda-package]

# Override built-in mapping
numpy:
  conda: [my-custom-numpy]

# Built-in package (no conda equivalent needed)
my-builtin-package:
  conda: []
```

## API Reference

### `PypiCondaMapper`

The main struct for performing package mappings.

#### Methods

- `new()` - Create mapper with built-in mappings
- `from_file(path)` - Create mapper from custom file only
- `with_custom_file(path)` - Merge custom file with built-in mappings
- `map_package(pypi_name)` - Map single package, returns `Option<&[String]>`
- `map_package_or_fallback(pypi_name)` - Map with fallback to original name
- `map_package_strict(pypi_name)` - Map single package, error on unmapped
- `map_packages(iter)` - Batch map multiple packages (fallback mode)
- `map_packages_strict(iter)` - Batch map multiple packages (strict mode)
- `has_mapping(pypi_name)` - Check if mapping exists
- `mapping_count()` - Get total number of mappings

## Automatic Dependency Conversion

The pixi-build-python backend automatically converts PyPI dependencies from `pyproject.toml` files to conda dependencies:

### Build System Requirements → Host Dependencies
```toml
# pyproject.toml
[build-system]
requires = ["setuptools>=45", "wheel", "numpy"]
```
Gets converted to conda host dependencies in the generated recipe.

### Project Dependencies → Run Dependencies  
```toml
# pyproject.toml
[project]
dependencies = [
    "numpy>=1.20.0",
    "torch",
    "requests"
]
```
Gets converted to conda run dependencies in the generated recipe:
- `numpy` → `numpy`
- `torch` → `pytorch` 
- `requests` → `requests`

### Error Handling
If PyPI packages have no conda mapping, the build will collect all unmapped packages and fail with a comprehensive error:

**Single unmapped package:**
```
Error: PyPI package 'unknown-package' has no conda mapping
Help: Add 'unknown-package' to the mapping file or to your pixi.toml dependencies
```

**Multiple unmapped packages:**
```
Error: Multiple PyPI packages have no conda mapping: unknown-pkg1, unknown-pkg2, unknown-pkg3
Help: Add these packages to the mapping file or to your pixi.toml dependencies
```

The system collects all unmapped packages from both `build-system.requires` and `project.dependencies` before reporting errors, providing a complete view of what needs to be addressed.

### License File Handling

The backend properly handles different license specifications in `pyproject.toml`:

| pyproject.toml Format | Generated Recipe |
|---|---|
| `license = { file = "LICENSE" }` | Only `license_file: "${{ SRC_DIR }}/LICENSE"` |
| `license = { text = "MIT" }` | Only `license: "MIT"` |
| `license = "MIT"` | Only `license: "MIT"` (SPDX identifier) |

License files use the `${{ SRC_DIR }}` template variable to reference the build source directory.

### Manual Integration
For custom use cases, you can also use the mapping system directly:

```rust
use pixi_build_python::pypi_conda_mapping::PypiCondaMapper;

let mapper = PypiCondaMapper::new();
let conda_packages = mapper.map_package_strict("torch")?; // Returns ["pytorch"]
```

## Running Examples

See the included example for a comprehensive demonstration:

```bash
cargo run --example pypi_mapping_demo
```

## Contributing

To add new mappings or improve existing ones:

1. Edit `pypi-to-conda-mapping.yaml`
2. Add appropriate test cases in `src/pypi_conda_mapping.rs`
3. Run tests: `cargo test`
4. Update this README if needed

## Testing

Run the test suite:
```bash
cargo test --package pixi-build-python
```

The tests cover:
- Built-in mapping loading
- Single and batch package mapping
- Custom mapping file loading
- Fallback behavior
- Edge cases (built-in packages, unknown packages)

