# Contributing to the Common Platform

Thank you for your interest in contributing to the RCR Common Platform! This guide will walk you through the process of making contributions to the project.

## Prerequisites

Before contributing, make sure you have:
- A GitHub account
- Git installed on your development machine
- Basic familiarity with Git and GitHub (see our [Git Guide](https://github.com/RoseCityRobotics/rcr-foundations) for detailed instructions)

## Contribution Workflow

### 1. Fork and Clone the Repository

First, fork the repository on GitHub, then clone your fork locally:

```bash
# Clone your fork (replace 'yourusername' with your GitHub username)
git clone https://github.com/yourusername/common_platform.git
cd common_platform

# Add the upstream repository
git remote add upstream https://github.com/RoseCityRobotics/common_platform.git
```

### 2. Create a New Branch

Create a new branch for your contribution:

```bash
# Create and switch to a new branch
git checkout -b feature/your-feature-name

# Or for bug fixes
git checkout -b fix/issue-description
```

**Branch Naming Conventions:**
- `feature/` - for new features
- `fix/` - for bug fixes
- `docs/` - for documentation updates
- `refactor/` - for code refactoring

### 3. Make Your Changes

Make your changes to the code, documentation, or other project files. For documentation changes:

- Edit the relevant `.md` files in the `docs/source/` directory
- Test your changes by building the documentation locally
- Ensure your changes follow the existing style and format

### 4. Commit Your Changes

Stage and commit your changes with a clear, descriptive message:

```bash
# Stage your changes
git add .

# Commit with a descriptive message
git commit -m "Add new feature: brief description of what you added"

# Or for documentation
git commit -m "docs: update setup instructions for new hardware"
```

**Commit Message Guidelines:**
- Use present tense ("Add feature" not "Added feature")
- Keep the first line under 50 characters
- Use the body to explain what and why, not how
- Reference issues with "Fixes #123" or "Closes #456"

### 5. Push to Your Fork

Push your branch to your GitHub fork:

```bash
# Push your branch to your fork
git push origin feature/your-feature-name
```

### 6. Create a Pull Request

1. **Go to your fork** on GitHub (https://github.com/yourusername/common_platform)
2. **Click "Compare & pull request"** for your new branch
3. **Fill out the PR template:**
   - **Title**: Clear, descriptive title
   - **Description**: Explain what your changes do and why
   - **Checklist**: Mark completed items
   - **Related Issues**: Link any related issues

4. **Submit the PR** and wait for review

## Types of Contributions

### Documentation Improvements
- Fix typos or grammatical errors
- Add missing information or procedures
- Improve clarity and organization
- Add new guides or tutorials

### Code Contributions
- Bug fixes
- New features
- Performance improvements
- Code refactoring

### Hardware Contributions
- PCB design improvements
- Component recommendations
- Assembly instructions
- Troubleshooting guides

## Review Process

1. **Automated Checks**: Your PR will run automated tests and checks
2. **Code Review**: Maintainers will review your changes
3. **Feedback**: Address any requested changes
4. **Approval**: Once approved, your changes will be merged

## Getting Help

If you need help with the contribution process:

- **Git Basics**: See our comprehensive [Git Guide](https://github.com/RoseCityRobotics/rcr-foundations)
- **Documentation**: Check existing documentation for examples
- **Issues**: Open an issue for questions or discussions
- **Discussions**: Use GitHub Discussions for general questions

## Code of Conduct

Please be respectful and constructive in all interactions. We're here to learn and build together!

## Thank You

Every contribution, no matter how small, helps improve the project for everyone. Thank you for contributing to the RCR Common Platform!
