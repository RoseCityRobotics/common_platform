# Module Library

This library contains individual procedural modules that can be composed into larger guides. Each module is a self-contained task or procedure that can be reused across different documentation pages.

## Setup and Configuration

Modules for initial robot setup and configuration.

```{toctree}
:maxdepth: 1
:titlesonly:

modules/connect-to-raspberry-pi
modules/pull-common-platform-updates
modules/configure-host-settings
modules/configure-network
modules/set-ros-namespacing
```

## Firmware and Hardware

Modules related to firmware updates and hardware operations.

```{toctree}
:maxdepth: 1
:titlesonly:

modules/flash-teensy
modules/startup-robot
```

## Robot Operations

Modules for operating the robot.

```{toctree}
:maxdepth: 1
:titlesonly:

modules/start-microros-agent
modules/monitor-teensy-serial
modules/start-keyboard-teleop
```

## SLAM and Mapping

Modules for simultaneous localization and mapping.

```{toctree}
:maxdepth: 1
:titlesonly:

modules/copy-cartographer-launch
modules/start-robot-state-and-cartographer
modules/save-map
modules/view-robot-slam
```

## Module Metadata

Each module includes:

- **Device indicator**: Shows whether the command runs on {{pi}} (Raspberry Pi) or {{pc}} (Development Computer)
- **Tags**: Categorize the module for easy searching
- **Description**: Clear explanation of what the module does
- **Code blocks**: Copy-pasteable commands with explanations

## Using Modules

Modules can be:

1. **Referenced individually** - Link directly to a specific module
2. **Composed into procedures** - Multiple modules combined into complete workflows
3. **Reused across guides** - Same module appears in multiple contexts without duplication

