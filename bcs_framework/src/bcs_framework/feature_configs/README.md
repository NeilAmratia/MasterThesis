# Feature Configuration Documentation

## Overview
Feature configuration files define which features are active and map base files to their corresponding delta files. This mapping enables efficient processing and variant generation.

## Directory Structure
feature_config/
├── door_system.toml
└── ... (other configuration files)


## Configuration File Format
[feature_delta_files]
feature_name = "path/to/delta.toml"

## Example
[feature_delta_files]
CLS = "deltas/CLS.toml"
Automatic_PW = "deltas/Automatic_PW.toml"
Heatable = "deltas/Heatable.toml"
LED_Heatable = "deltas/LED_Heatable.toml"

## Configuration Components

### Base File Mapping
- Specifies the target base file
- Provides relative path from project root
- Must correspond to existing base feature file

### Feature Definition
- Feature name (must match delta markers)
- Active state (true/false)
- List of associated delta files
- Optional dependencies and constraints

### Delta File References
- Relative paths to delta files
- Must match existing delta implementations
- Organized by feature

## Guidelines

### File Organization
1. One configuration file per base feature
2. Use consistent naming conventions
3. Maintain clear structure
4. Document feature relationships
5. Version control configurations

### Feature Management
1. Group related features
3. Document feature interactions
4. Maintain feature hierarchy
5. Consider feature combinations

### Best Practices
1. Keep configurations focused
2. Document constraints clearly
3. Validate configurations
4. Test feature combinations
5. Maintain traceability

## Error Prevention
1. Validate YAML syntax
2. Check file paths
3. Verify feature names
4. Test configurations
5. Document known issues

## Testing
1. Verify configuration loading
2. Test feature combinations
3. Validate dependency resolution
4. Check file references
5. Maintain test coverage

## Usage Instructions
1. Create configuration file for each base feature
2. Define active features and their delta files
3. Specify feature dependencies
4. Validate configuration
5. Generate variants