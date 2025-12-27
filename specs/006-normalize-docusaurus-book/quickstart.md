# Quickstart: Repository Cleanup & Normalization for Physical AI & Humanoid Robotics Textbook

## Setup Instructions

### Prerequisites
- Node.js v18+ (for Docusaurus)
- npm or yarn package manager
- Git for version control
- Python 3.8+ (for any preprocessing scripts)

### Initial Setup

1. Navigate to the repository:
```bash
cd /mnt/e/Hackathon-1/physical-ai-humanoid-robotics
```

2. The main Docusaurus textbook is located in the `my-book` directory:
```bash
cd my-book
```

3. Install dependencies:
```bash
npm install
```

4. Start the development server:
```bash
npm start
```

## Understanding the Current Structure

The repository currently has multiple documentation directories that need to be consolidated:

1. **Main textbook**: `/my-book/docs` - This contains the core textbook with 4 modules
2. **Old modules**: `/my-book/docs/tutorial-basics`, `/my-book/docs/tutorial-extras`, `/my-book/docs/ros2-fundamentals` - These need to be removed
3. **External docs**: `/docs`, `/gazebo_unity_simulation/documentation`, `/isaac_sim_vslam_nav/documentation` - These need to be evaluated for content migration
4. **Student materials**: `/student-materials` - May contain useful content for textbook

## Cleanup Process

### Step 1: Audit and Backup
Before making any changes, audit the content in all documentation directories:
```bash
# Check existing structure
find . -name "docs" -type d
ls -la my-book/docs/
```

### Step 2: Content Migration
1. Extract valuable content from external documentation directories
2. Integrate this content into appropriate chapters in the main textbook
3. Ensure content follows Docusaurus best practices (frontmatter, proper headings, links)

### Step 3: Directory Cleanup
Remove non-essential documentation directories:
```bash
# Remove old tutorial directories that are not needed
rm -rf my-book/docs/tutorial-basics
rm -rf my-book/docs/tutorial-extras
rm -rf my-book/docs/ros2-fundamentals
```

### Step 4: Navigation Update
Update the sidebar configuration (`sidebars.js`) to reflect the cleaned-up structure, ensuring only the 4 required modules remain.

## Key Locations

- **Main textbook**: `/my-book/docs/`
- **Module 1 - ROS2**: `/my-book/docs/module-1-ros2/`
- **Module 2 - Digital Twin**: `/my-book/docs/module-2-digital-twin/`
- **Module 3 - Isaac**: `/my-book/docs/module-3-isaac/`
- **Module 4 - VLA**: `/my-book/docs/module-4-vla/`
- **Navigation**: `/my-book/sidebars.js`
- **Configuration**: `/my-book/docusaurus.config.js`

## Testing the Cleanup

After cleanup:

1. Verify all 4 modules are accessible from the sidebar:
   - Module 1: ROS2 Fundamentals
   - Module 2: Digital Twins (Gazebo & Unity)
   - Module 3: AI-Robot Brain (NVIDIA Isaac)
   - Module 4: Vision-Language-Action (VLA) Models

2. Check that all internal links work correctly

3. Verify that search functionality works across all chapters

4. Test responsive design on different screen sizes

## Success Criteria

After completing the cleanup:
- Only one documentation structure exists under `/my-book/docs/`
- The structure contains exactly 4 modules with 4 chapters each
- Sidebar navigation shows only the 4 main modules
- No duplicate or redundant content exists
- All content renders correctly in the Docusaurus site
- External documentation directories have been removed or consolidated appropriately