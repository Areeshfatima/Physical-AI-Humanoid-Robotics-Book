# Research: Repository Cleanup & Normalization for Physical AI & Humanoid Robotics Textbook

## Decision: Repository Structure Audit
**Rationale**: Identified multiple documentation directories that need consolidation:
- `/my-book/docs` - Current textbook with 4 modules (main target)
- `/docs` - Contains additional content (e.g., vla-system/)
- `/gazebo_unity_simulation/documentation/docs` - Gazebo/Unity documentation
- `/isaac_sim_vslam_nav/documentation/docs` - Isaac simulation documentation
- `/student-materials/docs` - Student materials documentation
- Old tutorial directories in `/my-book/docs` (tutorial-basics, tutorial-extras, ros2-fundamentals)

## Decision: Content Migration Strategy
**Rationale**: Consolidate all relevant educational content into the main Docusaurus textbook under `/my-book/docs` with 4 modules and 4 chapters each, removing all non-book, duplicate or mislocated documentation.

## Decision: Module Organization
**Rationale**: The existing 4 modules in `/my-book/docs` already match the requirements:
- `module-1-ros2` - ROS2 Fundamentals
- `module-2-digital-twin` - Digital Twin (Gazebo & Unity)
- `module-3-isaac` - AI-Robot Brain (NVIDIA Isaac)
- `module-4-vla` - Vision-Language-Action Models

## Decision: Cleanup Scope
**Rationale**: Remove documentation directories outside of the main textbook that are not required for the core educational textbook:
- Remove `/docs/vla-system/` (appears to be redundant content)
- Remove duplicate/alternative module directories in `/my-book/docs/`
- Clean up old tutorial content
- Preserve the main textbook under `/my-book/docs/`

## Decision: Docusaurus Configuration
**Rationale**: The `my-book` directory is already configured as a proper Docusaurus site with the correct sidebar structure for the 4 required modules.

## Decision: Migration Approach
**Rationale**: 
1. Extract valuable content from external documentation directories
2. Integrate this content into the appropriate chapters in the main textbook
3. Remove redundant or non-essential documentation
4. Maintain proper navigation and linking

## Alternatives Considered
1. **Complete rewrite**: Creating textbook from scratch was rejected because significant content already exists
2. **Separate documentation sites**: Keeping multiple documentation sites would create fragmented user experience
3. **Partial consolidation**: Only migrating some content would still leave fragmented documentation