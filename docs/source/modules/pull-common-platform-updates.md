---
type: module
slug: pull-common-platform-updates
title: Pull the latest changes from GitHub
description: Update your common_platform repository with the latest code from GitHub.
tags: [git, setup, update]
device: pi
---

# Pull the latest changes from GitHub

{{pi}} **On the Raspberry Pi**

## Navigate to the repository directory

```bash
cd repos/common_platform
```

## Stash any local changes

```bash
git stash
```

## Update the Git remote (one-time setup)

Update the Git remote for the public repo so you don't need credentials. You should only need to do this step once.

```bash
git remote set-url origin "https://github.com/RoseCityRobotics/common_platform.git"
```

## Pull the latest changes

```bash
git pull origin main
```

## Re-apply your stashed changes

```bash
git stash apply
```

:::{tip}
If you have conflicts after applying your stash, you'll need to resolve them manually before proceeding.
:::

