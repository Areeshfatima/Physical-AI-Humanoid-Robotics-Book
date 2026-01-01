---
title: "Introduction to ROS2 Architecture"
sidebar_position: 1
id: "module-1-chapter-1"
---

# Introduction to ROS2 Architecture

## Overview

This chapter introduces the architecture of ROS2 (Robot Operating System 2), which is fundamentally different from its predecessor, ROS 1. Understanding the architecture is key to effectively using ROS2 for robotics applications.

## Learning Objectives

After completing this chapter, you will be able to:
- Explain the DDS-based architecture of ROS2
- Identify the main components of ROS2
- Understand the role of RMW (ROS Middleware Interface)
- Compare ROS1 and ROS2 architectures

## What is ROS2?

ROS2 is a collection of software frameworks that provide functionality for developing robot applications. Unlike ROS1, which used a custom distributed messaging system, ROS2 is built on DDS (Data Distribution Service), an industry standard for distributed systems.

The main components of ROS2 include:
- Nodes: Processes that perform computation
- Topics: Named buses over which nodes exchange messages
- Services: Synchronous request/response communication
- Actions: Asynchronous goal-oriented communication with feedback
- Parameters: Configuration values that can be changed at runtime
- Launch files: XML/YAML files that start multiple nodes at once

## DDS and RMW

DDS (Data Distribution Service) provides the underlying communication layer for ROS2. The ROS Middleware (RMW) interface allows ROS2 to work with different DDS implementations. This architecture provides:
- Real-time performance
- Scalability
- Security features
- Multi-language support

## Summary

This chapter provided an overview of the ROS2 architecture, which forms the foundation for all subsequent learning in this module. The DDS-based architecture provides significant improvements over ROS1 in terms of real-time performance, scalability, and security.

[Next Chapter: ROS2 Packages and Workspaces](./chapter-2)