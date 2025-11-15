# Procedures

Procedures are complete workflows that guide you through common tasks by composing individual modules from the [Module Library](../modules/index.md).

## Getting Started

Start here if you're setting up a robot for the first time.

```{toctree}
:maxdepth: 1
:titlesonly:

raspberry-pi-setup
firmware-and-teleop
```

## SLAM and Navigation

Procedures for mapping and autonomous navigation.

```{toctree}
:maxdepth: 1
:titlesonly:

slam-mapping
```

## About Procedures

Each procedure:
- Combines multiple modules into a complete workflow
- Shows the proper sequence of steps
- Includes prerequisites and next steps
- References the underlying modules for detailed information

## Difference Between Modules and Procedures

- **Modules** (in `/modules/`) are atomic, reusable tasks (e.g., "Flash Teensy", "Configure Network")
- **Procedures** (in `/procedures/`) are complete workflows that compose multiple modules (e.g., "Raspberry Pi Setup", "SLAM Mapping")

Think of modules as LEGO bricks and procedures as the instruction manual showing how to build something specific.

