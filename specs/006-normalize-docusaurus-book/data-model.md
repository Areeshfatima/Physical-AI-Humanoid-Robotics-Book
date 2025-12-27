# Data Model: Educational Textbook for Physical AI & Humanoid Robotics

## Entities

### Module
- **id**: String (unique identifier for the module, e.g., "module-1-ros2")
- **title**: String (display title of the module)
- **description**: String (brief description of module content)
- **position**: Integer (order of the module in the textbook: 1-4)
- **chapters**: Array of Chapter references (exactly 4 chapters per module)
- **metadata**: Object (creation date, last updated, author information)

### Chapter
- **id**: String (unique identifier for the chapter, e.g., "module-1-ros2-chapter-1")
- **title**: String (display title of the chapter)
- **module_id**: String (reference to parent module)
- **position**: Integer (order of the chapter within the module: 1-4)
- **estimated_reading_time**: Integer (in minutes)
- **learning_objectives**: Array of Strings (what students will learn)
- **content**: String (the actual chapter content in Markdown/MDX format)
- **prerequisites**: Array of Strings (knowledge needed before reading)
- **exercises**: Array of Objects (practice problems or activities)
- **resources**: Array of Objects (external resources linked to the chapter)
- **metadata**: Object (creation date, last updated, author information)

### Section
- **id**: String (unique identifier for the section)
- **title**: String (title of the section)
- **chapter_id**: String (reference to parent chapter)
- **position**: Integer (order within the chapter)
- **content**: String (the actual section content)
- **heading_level**: Integer (HTML heading level: H1-H4)
- **media_assets**: Array of Objects (images, videos, diagrams referenced in the section)

### LearningObjective
- **id**: String (unique identifier)
- **description**: String (specific learning objective)
- **chapter_id**: String (which chapter this objective belongs to)
- **blooms_level**: String (cognitive level per Bloom's taxonomy: remember, understand, apply, analyze, evaluate, create)
- **assessment_method**: String (how the objective will be assessed)

### Exercise
- **id**: String (unique identifier for the exercise)
- **chapter_id**: String (reference to the chapter containing this exercise)
- **type**: String (e.g., "conceptual", "implementation", "project", "quiz")
- **difficulty**: String (e.g., "basic", "intermediate", "advanced")
- **problem_statement**: String (the exercise question/content)
- **solution_outline**: String (outline of the expected solution)
- **evaluation_criteria**: Array of Strings (how the exercise will be evaluated)

### MediaAsset
- **id**: String (unique identifier for the media asset)
- **url**: String (relative path to the asset file)
- **type**: String (e.g., "image", "video", "diagram", "equation", "code_example")
- **caption**: String (description of the media content)
- **alt_text**: String (accessibility text for images)
- **position**: Integer (where in the chapter/section this asset appears)
- **chapter_id**: String (which chapter contains this asset)

### UserProgress
- **id**: String (unique identifier for the progress record)
- **user_id**: String (identifier for the user)
- **chapter_id**: String (reference to the chapter being tracked)
- **completion_status**: String (enum: "not_started", "in_progress", "completed")
- **last_accessed**: DateTime (timestamp of last access)
- **progress_percentage**: Integer (0-100)
- **notes**: String (optional user notes)

### User
- **id**: String (unique identifier for the user)
- **username**: String (unique username)
- **email**: String (user's email address)
- **role**: String (enum: "reader", "contributor", "editor", "admin")
- **created_at**: DateTime (account creation timestamp)
- **last_login**: DateTime (timestamp of last login)

### NavigationBreadcrumb
- **id**: String (unique identifier for the breadcrumb entry)
- **current_page_id**: String (the page the user is currently viewing)
- **parent_pages**: Array of Objects (hierarchy of parent pages)
- **navigation_path**: Array of Strings (sequence of page titles from root to current)