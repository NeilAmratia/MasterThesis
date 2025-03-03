# Delta Features Documentation

## Overview
Delta features define modifications to be applied to base features. They are organized using TOML files that specify how the code should be modified based on feature configurations.

## Directory Structure
deltas/
├── CLS.toml         # Central Locking System deltas
├── Automatic_PW.toml # Automatic Power Window deltas
├── README.md        # This documentation
└── ...             # Other delta files

## Delta File Format (TOML)
Delta files use TOML format to define modifications in a structured manner:
- all the delta must start with [[deltas]] 
- delta type should be specifiec: add, delete or change
- all deltas should have specific ID number 1,2, etc.
- deltas will only be applied if the condition or conditions are met
- content will assume the identation of the marker and the toml file multiple line code indentation is followed when the deltas are applied, so please keep the indentation as required in deltas in content

### Standard delta file format with single condition (TOML)
[[deltas]]
type = "<operation_type>"
id = <unique_identifier>
condition = { feature = "<feature_name>", state = "<active/inactive>" }
content = """
<code_to_insert> // multiple lines of code can be inserted with required indentation
"""

### Standard delta file format with multiple condition (TOML)
[[deltas]]
type = "<operation_type>"
id = <unique_identifier>
condition = { 
    and = [
        { feature = "<feature_name>", state = "<active/inactive>" },
        { feature = "<feature_name>", state = "<active/inactive>" }
    ]
}
content = """
<code_to_insert> // multiple lines of code can be inserted with required indentation
"""

## Delta Types

### 1. Add Delta
- Introduces new functionality
- Requires code content
[[deltas]]
type = "add"
id = <unique_identifier>
condition = { feature = "<feature_name>", state = "<active/inactive>" }
content = """
<code_to_insert> // multiple lines of code can be inserted with required indentation
"""

### 2. Delete Delta
- Removes existing code
- No content required
- Requires matching END marker in base file
[[deltas]]
type = "delete"
id = <unique_identifier>
condition = { feature = "<feature_name>", state = "<active/inactive>" }


### 3. Change Delta
- Modifies existing code
- Requires new code content
- Requires matching END marker in base file
[[deltas]]
type = "change"
id = <unique_identifier>
condition = { feature = "<feature_name>", state = "<active/inactive>" }
content = """
<code_to_insert> // multiple lines of code can be inserted with required indentation
"""

## Guidelines

### File Organization
1. Group related deltas by feature
2. Use consistent file naming
3. Maintain clear directory structure
4. Document delta dependencies
5. Version control delta files

### Content Management
1. Preserve code formatting
2. Use meaningful identifiers
3. Document complex conditions
4. Maintain code readability
5. Consider feature interactions

### Best Practices
1. Keep deltas focused and minimal
2. Document conditions clearly
3. Test deltas independently
4. Validate delta combinations
5. Maintain traceability

## Error Prevention
1. Validate TOML syntax
2. Check ID correspondence
3. Verify condition logic
4. Test delta applications
5. Document known issues

## Testing
1. Test each delta individually
2. Verify condition evaluation
3. Test feature combinations
4. Validate generated code
5. Maintain test coverage