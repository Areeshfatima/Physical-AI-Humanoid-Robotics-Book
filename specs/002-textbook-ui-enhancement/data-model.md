# Data Model: Docusaurus Textbook UI Enhancement

## Overview
This document defines the data model for the Physical AI Humanoid Robotics textbook website UI enhancement. It focuses on the content structure, navigation elements, and UI components that need to be implemented.

## Key Entities

### 1. Logo Asset
- **Entity Name**: LogoAsset
- **Fields**:
  - id: string (unique identifier)
  - path: string (path to image file in static assets)
  - altText: string (accessibility alt text)
  - dimensions: object {width: number, height: number}
  - format: string (e.g., 'svg', 'png', 'webp')
  - responsiveVariants: array of objects {breakpoint: string, path: string}

- **Relationships**: None
- **Validation Rules**:
  - Path must be a valid static asset path
  - Alt text must be provided for accessibility
  - File format must be supported by browsers

### 2. Sidebar Navigation
- **Entity Name**: SidebarNavigation
- **Fields**:
  - id: string (unique identifier for navigation structure)
  - items: array of objects {id: string, label: string, href: string, children: array}
  - ordering: array (specific order: ['Introduction', 'Module 1', 'Module 2', 'Module 3', 'Module 4'])
  - activeItem: string (currently selected item)

- **Relationships**: Contains multiple NavigationItem
- **Validation Rules**:
  - Must contain exactly the required items in specified order
  - No auto-generated items allowed
  - All navigation links must be valid

### 3. Hero Section Content
- **Entity Name**: HeroSection
- **Fields**:
  - id: string
  - title: string (main heading)
  - subtitle: string (subheading or description)
  - images: array of objects {path: string, alt: string, caption: string}
  - ctaButton: object {label: string, link: string}
  - theme: string (style theme for the section)

- **Relationships**: Contains multiple ImageAsset
- **Validation Rules**:
  - Title and subtitle must be present
  - Images must be relevant to robotics/AI
  - All images must have appropriate alt text

### 4. GitHub Link
- **Entity Name**: GitHubLink
- **Fields**:
  - id: string
  - url: string ('https://github.com/Areeshfatima/Physical-AI-Humanoid-Robotics-Book.git')
  - label: string ('GitHub')
  - icon: string (icon class or path)
  - location: enum ('navbar', 'footer')

- **Relationships**: None
- **Validation Rules**:
  - URL must match the specified GitHub repository
  - Location must be either 'navbar' or 'footer'

### 5. Theme Configuration
- **Entity Name**: ThemeConfig
- **Fields**:
  - id: string
  - colors: object {primary: string, secondary: string, background: string, text: string}
  - typography: object {fontFamily: string, headings: object, body: object}
  - breakpoints: object {mobile: string, tablet: string, desktop: string}
  - accessibility: object {contrastMode: boolean, fontSize: string}

- **Relationships**: Applied to all UI components
- **Validation Rules**:
  - All color combinations must meet WCAG 2.1 AA contrast requirements
  - Typography must be readable and accessible
  - Responsive breakpoints must cover all device sizes

## State Transitions

### Sidebar Navigation State
- **Initial State**: Navigation structure is loaded
- **Transition 1**: User hovers over navigation item → Visual feedback (highlight)
- **Transition 2**: User clicks navigation item → Active state updates, page loads
- **Transition 3**: Page loads → Active state reflects current page

### Hero Section State
- **Initial State**: Hero content is loaded with default image
- **Transition 1**: On page load → Images load with appropriate fallbacks
- **Transition 2**: On responsive change → Appropriate image variant is displayed

## Relationships

### Content-to-UI Relationships
1. NavigationItem → ContentPage
   - Each navigation item links to a specific content page
   - The ordering of navigation items reflects the educational flow

2. ImageAsset → ContentPage
   - Hero section images are contextually related to the textbook content
   - Images should support the educational objectives

3. ThemeConfig → All UI Components
   - Theme configuration is applied uniformly across all UI elements
   - Ensures consistent look and feel throughout the textbook site