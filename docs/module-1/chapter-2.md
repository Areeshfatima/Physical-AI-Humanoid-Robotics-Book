---
title: "ROS2 Packages and Workspaces"
sidebar_position: 2
id: "module-1-chapter-2"
---

# ROS2 Packages and Workspaces

## Overview

This chapter explores the fundamental organizational structures in ROS2: packages and workspaces. These structures provide a way to organize code, share resources, and build complex robotics applications.

## Learning Objectives

After completing this chapter, you will be able to:
- Create and manage ROS2 packages
- Understand the structure of a ROS2 workspace
- Build packages using colcon
- Manage dependencies between packages
- Use common package templates

## What is a Package?

A package is the basic building block of a ROS2 application. It contains:

- Source code (C++ or Python)
- Launch files
- Configuration files
- Resource files (models, images, etc.)
- Build files (CMakeLists.txt, setup.py)
- Package manifest (package.xml)

A package must contain a package.xml file that describes the package, its dependencies, and its maintainers.

## What is a Workspace?

A workspace is a directory that contains one or more ROS2 packages. The typical workspace structure includes:

- src/: Where source code packages are placed
- build/: Where intermediate build files are stored
- install/: Where built packages are installed
- log/: Where log files are stored

## Creating a Package

To create a new package, use the `ros2 pkg create` command:

```bash
ros2 pkg create --build-type ament_cmake <package_name>
```

Or for Python packages:

```bash
ros2 pkg create --build-type ament_python <package_name>
```

This creates a skeleton package with the necessary files and directories.

## Building a Workspace

To build a workspace, navigate to the workspace root directory and run:

```bash
colcon build
```

This command:
- Discovers all packages in the src directory
- Builds each package according to its build type
- Installs packages to the install directory

## Summary

This chapter covered the organizational structures of ROS2: packages and workspaces. Understanding these structures is crucial for organizing complex robotics applications.

[Next Chapter: Nodes, Topics, Services and Actions](./chapter-3)