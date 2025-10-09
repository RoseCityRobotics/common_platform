# Robot Operations Documentation

This directory contains the robot operations documentation for the RCR Common Robotics Platform. The documentation is integrated into the main Sphinx documentation system and can be built alongside the rest of the project documentation.

## File Structure

```
docs/source/operations/
├── index.md                      # Main operations manual index
├── setup/                        # Initial setup and configuration
│   ├── index.md
│   ├── hardware_setup.md
│   ├── software_setup.md
│   └── calibration.md
├── daily_operations/             # Routine operations
│   ├── index.md
│   ├── startup_procedure.md
│   ├── shutdown_procedure.md
│   └── maintenance.md
├── navigation/                   # Navigation-related operations
│   ├── index.md
│   ├── mapping.md
│   ├── localization.md
│   └── path_planning.md
├── sensors/                      # Sensor operations
│   ├── index.md
│   ├── lidar_operations.md
│   ├── camera_operations.md
│   └── imu_operations.md
├── troubleshooting/              # Problem-solving guides
│   ├── index.md
│   ├── common_issues.md
│   ├── error_codes.md
│   └── recovery_procedures.md
└── advanced/                     # Advanced operations
    ├── index.md
    ├── custom_controllers.md
    └── firmware_updates.md
```

## Building the Documentation

### Prerequisites

1. Install Python virtual environment support:
   ```bash
   sudo apt install python3.12-venv
   ```

2. Create and activate a virtual environment:
   ```bash
   cd docs
   python3 -m venv venv
   source venv/bin/activate
   ```

3. Install documentation dependencies:
   ```bash
   pip install -r requirements.txt
   ```

### Building

1. Build the HTML documentation:
   ```bash
   make html
   ```

2. View the documentation:
   ```bash
   # Open in browser
   firefox build/html/index.html
   ```

### Alternative Build Methods

If you prefer not to use a virtual environment:

1. Install system packages:
   ```bash
   sudo apt install python3-sphinx python3-sphinx-rtd-theme
   pip install myst-parser --break-system-packages
   ```

2. Build directly:
   ```bash
   sphinx-build -b html source build
   ```

## Adding New Operations

### Creating New Operation Files

1. **Choose the appropriate category** (setup, daily_operations, navigation, etc.)
2. **Create the markdown file** with a descriptive name
3. **Update the category index.md** to include the new file
4. **Follow the established format** (see existing files for examples)

### File Naming Conventions

- Use lowercase with underscores: `hardware_setup.md`
- Be descriptive: `lidar_operations.md` not `lidar.md`
- Use consistent terminology across files

### Content Guidelines

1. **Structure**: Use clear headings and sections
2. **Safety**: Always include safety warnings where appropriate
3. **Prerequisites**: List what's needed before starting
4. **Step-by-step**: Provide clear, numbered procedures
5. **Troubleshooting**: Include common issues and solutions
6. **Cross-references**: Link to related procedures

### Example File Structure

```markdown
# Operation Title

Brief description of what this operation does.

## Prerequisites

- List of requirements
- Tools needed
- Knowledge prerequisites

## Procedure

### Step 1: Preparation
Detailed steps...

### Step 2: Execution
Detailed steps...

## Verification

How to verify the operation was successful.

## Troubleshooting

Common issues and solutions.

## Safety Notes

⚠️ Important safety information.

## Related Operations

- [Link to related procedure](link.md)
```

## Integration with Main Documentation

The operations documentation is automatically included in the main documentation build through the `index.rst` file, which includes:

```rst
.. toctree::
   :maxdepth: 2
   :caption: Contents:

   intro
   hardware
   software
   testing
   operations/index
```

This means the operations manual will appear as a top-level section in the generated documentation.

## Maintenance

### Regular Updates

- Review procedures quarterly
- Update when hardware/software changes
- Incorporate user feedback
- Keep safety information current

### Version Control

- Commit changes with descriptive messages
- Tag major updates
- Maintain change log
- Document breaking changes

## Contributing

To contribute to the operations documentation:

1. **Fork the repository**
2. **Create a feature branch**
3. **Make your changes**
4. **Test the documentation build**
5. **Submit a pull request**

### Review Process

- Technical accuracy review
- Safety review
- Usability testing
- Documentation standards compliance

---

*This documentation structure provides a comprehensive, maintainable system for robot operations that integrates seamlessly with your existing Sphinx documentation system.*
