# Data Model: Educational Textbook for Physical AI & Humanoid Robotics

## Entities

### Module
- **id**: String (unique identifier for the module)
- **title**: String (title of the module)
- **description**: String (brief description of the module content)
- **chapters**: Array of Chapter references (4 chapters per module as specified)
- **position**: Integer (order of the module in the textbook)
- **metadata**: Object (creation date, last updated, author info)

### Chapter
- **id**: String (unique identifier for the chapter)
- **title**: String (title of the chapter)
- **module_id**: String (reference to the parent module)
- **content**: String (the actual chapter content in Markdown format)
- **position**: Integer (order of the chapter within the module)
- **estimated_reading_time**: Integer (in minutes)
- **objectives**: Array of Strings (learning objectives)
- **media_assets**: Array of Objects (references to images, videos, diagrams)
- **metadata**: Object (creation date, last updated, author info)

### Section
- **id**: String (unique identifier for the section)
- **title**: String (title of the section)
- **chapter_id**: String (reference to the parent chapter)
- **content**: String (the actual section content in Markdown format)
- **position**: Integer (order of the section within the chapter)
- **heading_level**: Integer (HTML heading level: 1-6)

### User Progress
- **id**: String (unique identifier for the progress record)
- **user_id**: String (identifier for the user)
- **chapter_id**: String (reference to the chapter being tracked)
- **status**: String (enum: 'in-progress', 'completed')
- **last_accessed**: DateTime (timestamp of last access)
- **progress_percentage**: Integer (0-100)
- **notes**: String (optional user notes on the chapter)

### User
- **id**: String (unique identifier for the user)
- **username**: String (unique username)
- **email**: String (user's email address)
- **role**: String (enum: 'reader', 'editor', 'admin')
- **created_at**: DateTime (account creation timestamp)
- **last_login**: DateTime (timestamp of last login)

### Content Editor
- **id**: String (unique identifier)
- **user_id**: String (reference to the User)
- **permissions**: Array of Strings (what content they can edit)
- **last_content_update**: DateTime (timestamp of last content modification)

### Log Entry
- **id**: String (unique identifier for the log entry)
- **timestamp**: DateTime (when the action occurred)
- **user_id**: String (identifier for the user, if authenticated)
- **action**: String (what action was taken: 'view_content', 'search', 'export_pdf', etc.)
- **target_id**: String (what content was acted upon)
- **target_type**: String (type of content: 'module', 'chapter', 'section')
- **ip_address**: String (IP address of the request)
- **user_agent**: String (browser/device information)