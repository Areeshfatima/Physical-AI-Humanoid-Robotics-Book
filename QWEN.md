# physical-ai-humanoid-robotics Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-10

## Active Technologies
- Python 3.10 (for ROS 2 Humble compatibility), C# (for Unity), XML/URDF/SDF (for robot models) + Gazebo/Ignition physics engine, Unity 3D, ROS 2 Humble Hawksbill, Unity Robotics Simulation Package, URDF/SDF parsers (003-digital-twin-gazebo-unity)
- N/A (simulation environments and robot models stored as files) (003-digital-twin-gazebo-unity)
- Python 3.10 (for ROS 2 Humble compatibility) + OpenAI Whisper (speech recognition), Hugging Face transformers (LLM), ROS 2 Humble (robotics framework), rclpy (Python ROS 2 client library) (005-vla-systems)
- N/A (real-time processing system) (005-vla-systems)
- JavaScript/TypeScript, Node.js (for Docusaurus), Python 3.10 (for AI/ML components) + Docusaurus v3.9.2, React, Node.js, OpenAI SDK, FastAPI, Qdrant (001-docusaurus-textbook)
- Git-based Markdown files for content, Qdrant for vector storage, Neon Postgres for metadata (001-docusaurus-textbook)
- JavaScript/TypeScript, Node.js 18+ (for Docusaurus), Python 3.10 (for content processing scripts) + Docusaurus v3.9.2, React, Node.js, OpenAI SDK (for RAG functionality), FS-extra (for file operations) (006-normalize-docusaurus-book)
- Git-based Markdown files for content in `/my-book/docs/`, with additional content potentially migrated from external documentation directories (006-normalize-docusaurus-book)
- JavaScript/TypeScript, Node.js 18+ (for Docusaurus compatibility) + Docusaurus CLI, docusaurus/core, docusaurus/preset-classic, React, Node.js, npm (001-docusaurus-cli-setup)
- File-based (Markdown documentation in /my-book/docs/) (001-docusaurus-cli-setup)
- JavaScript/Node.js (for processing markdown files) + fs (file system), yaml (YAML parsing), glob (file pattern matching) (001-markdown-front-matter-fix)
- JavaScript/Node.js (for processing markdown files) + JavaScript 18+ for Docusaurus compatibility + fs (file system), yaml (YAML parsing), glob (file pattern matching), js-yaml (YAML processing) (001-markdown-front-matter-fix)
- N/A (file processing system, operates on markdown files in docs directory) (001-markdown-front-matter-fix)
- JavaScript/Node.js (for processing markdown files) + JavaScript 18+ for Docusaurus compatibility + fs (file system), yaml (YAML parsing), glob (file pattern matching), js-yaml (YAML processing), docusaurus (v3.9.2) (010-repair-docusaurus-sidebar)
- JavaScript/TypeScript, Node.js 18+ (for Docusaurus compatibility) + Docusaurus v3.9.2, React, Node.js, npm, CSS (011-branding-theme-spec)
- N/A (configuration and assets stored as files) (011-branding-theme-spec)
- JavaScript/Node.js 18+ (for Docusaurus compatibility) + CSS/SASS for styling + Docusaurus v3.9.2, React, Node.js, npm, CSS (001-branding-fix)
- JavaScript/TypeScript, Node.js (for Docusaurus), Python 3.10 (for AI/ML components) + Docusaurus v3.9.2, React, Node.js, npm, CSS (for styling) (002-textbook-ui-enhancement)

- Python 3.10 (for ROS 2 Humble compatibility) + ROS 2 Humble Hawksbill, rclpy, rviz2, gazebo, URDF (001-ros2-fundamentals)

## Project Structure

```text
src/
tests/
```

## Commands

cd src [ONLY COMMANDS FOR ACTIVE TECHNOLOGIES][ONLY COMMANDS FOR ACTIVE TECHNOLOGIES] pytest [ONLY COMMANDS FOR ACTIVE TECHNOLOGIES][ONLY COMMANDS FOR ACTIVE TECHNOLOGIES] ruff check .

## Code Style

Python 3.10 (for ROS 2 Humble compatibility): Follow standard conventions

## Recent Changes
- 012-branding-ui-enhancement: Added JavaScript/TypeScript, Node.js 18+ (for Docusaurus compatibility) + Docusaurus v3.9.2, React, Node.js, npm
- 002-textbook-ui-enhancement: Added JavaScript/TypeScript, Node.js (for Docusaurus), Python 3.10 (for AI/ML components) + Docusaurus v3.9.2, React, Node.js, npm, CSS (for styling)
- 001-branding-fix: Added JavaScript/Node.js 18+ (for Docusaurus compatibility) + CSS/SASS for styling + Docusaurus v3.9.2, React, Node.js, npm, CSS


<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->
