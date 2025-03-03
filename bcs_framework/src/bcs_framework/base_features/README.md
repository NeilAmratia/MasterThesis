# Base Features Documentation

## Overview
Base features represent the core functionality of the software system, serving as the foundation for creating different variants. These files contain delta markers that indicate where modifications can be applied to generate specific variants.

## Directory Structure
base_features/
├── door_system.hpp
├── door_system.cpp
└── ... (other base feature files)

## Delta Markers
Base files are annotated with delta markers that specify locations for potential modifications. These markers are structured annotations that indicate:
- Type of modification (add/delete/change)
- Associated feature
- Unique identifier for tracking

### Delta Marker Format: 
For delete/change operations:
    // DELTA:<type>:<id>:<feature_name>
    [affected code]
    // DELTA:<type>:<id>:<feature_name>:END  (for delete/change operations)

For add operations (no END statement needed)
    // DELTA:<type>:<id>:<feature_name>
    [base code]

### Example Base File with Delta Markers
class DoorSystem : public rclcpp::Node {
public:
    DoorSystem();

private:
    class PowerWindow {
    public:
        // DELTA:change:2:Automatic_PW
        void processManual();
        // DELTA:change:2:Automatic_PW:END

        // DELTA:delete:1:CLS
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cls_lock;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cls_unlock;
        // DELTA:delete:1:CLS:END
    };
};

## Guidelines for Base Features

### Best Practices
1. Keep base features minimal and focused on core functionality
2. Use clear and consistent naming conventions
3. Document all delta markers thoroughly
4. Maintain clear separation of concerns
5. Follow ROS2 coding standards

### Delta Marker Placement
1. Place markers immediately before affected code
2. Use END markers for delete and change operations
3. Maintain consistent indentation
4. Keep markers and affected code in the same scope
5. Avoid overlapping markers

### Development Tips
1. Start with essential functionality
2. Plan extension points carefully
3. Consider feature interactions
4. Document dependencies
5. Test base functionality independently

## Error Prevention
1. Verify marker syntax
2. Check marker ID uniqueness
3. Ensure matching END markers
4. Validate feature names
5. Maintain code readability

## Testing
1. Test base functionality without any deltas
2. Verify all delta marker positions
3. Ensure code compiles without modifications
4. Document test cases for base features
5. Maintain separate test suites for base functionality