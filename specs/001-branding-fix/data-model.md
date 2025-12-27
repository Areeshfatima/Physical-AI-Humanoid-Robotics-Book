# Data Model: Docusaurus Branding Fix

## Entities

### Brand Elements
**Description**: Visual, textual, and metadata components that represent the textbook's identity
**Fields**:
- `logoImage`: String - Path to the logo image file
- `brandName`: String - Name of the textbook/brand
- `favicon`: String - Path to the favicon file
- `primaryColor`: String - Main brand color in hex format
- `secondaryColor`: String - Secondary brand color in hex format
- `metadata`: Object - Title, description, keywords for SEO

### Navigation Structure
**Description**: Organized hierarchy of content that enables users to navigate exactly: Module 1, Module 2, Module 3, Module 4 in the specified sequence
**Fields**:
- `items`: Array - List of navigation items in the correct order
- `moduleTitle`: String - Title of each module
- `modulePath`: String - Path to each module's content
- `moduleType`: String - Type of navigation item (category, link, etc.)
- `isVisible`: Boolean - Whether the item should appear in navigation

### Static Assets
**Description**: All images, logo, and other files that need to be properly referenced and rendered without fallback to alt text
**Fields**:
- `assetPath`: String - File path relative to static directory
- `assetType`: String - Type of asset (image, document, etc.)
- `altText`: String - Accessible alternative text for images
- `size`: Object - Dimensions (width, height) for images
- `optimized`: Boolean - Whether asset is optimized for web
- `fallback`: String - Path to fallback asset if primary fails

## Validation Rules

### Brand Elements Validation
- `logoImage` must be a valid path to an image file
- `brandName` must be non-empty and ASCII-safe
- `favicon` must be a valid path to a favicon file
- `primaryColor` and `secondaryColor` must be valid hex color codes

### Navigation Structure Validation
- `items` array must contain exactly 4 modules in specified order
- Each module must have a valid path to content
- `isVisible` must be true for required modules
- No extra items beyond the specified 4 modules

### Static Assets Validation
- `assetPath` must point to an existing file
- `assetType` must be a supported media type
- `altText` must be provided for all images
- `optimized` must be true for all images (to ensure 3 second load time)

## State Transitions

### Branding Implementation States
- `DEFAULT_STATE` → `CONFIGURATION`: When initial configuration is being set up
- `CONFIGURATION` → `CUSTOMIZATION`: When custom branding elements are being added
- `CUSTOMIZATION` → `VALIDATION`: When the branding implementation is being validated
- `VALIDATION` → `DEPLOYMENT`: When validated branding is ready for production
- `DEPLOYMENT` → `MONITORING`: When branding is live and being monitored for issues

## Relationships

### Brand Elements ↔ Navigation Structure
- Brand elements provide the visual identity for navigation components
- Navigation structure displays brand identity consistently throughout the site

### Navigation Structure ↔ Static Assets  
- Navigation items may reference static assets (like icons or images)
- Static assets support the visual presentation of navigation elements

### Brand Elements ↔ Static Assets
- Brand elements include static assets like logos and favicons
- Static assets embody the visual aspects of the brand identity