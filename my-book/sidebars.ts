import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  "textbookSidebar": [
    {
      "type": "doc",
      "id": "test-no-frontmatter"
    },
    {
      "type": "doc",
      "id": "test-invalid-frontmatter"
    },
    {
      "type": "doc",
      "id": "intro"
    },
    {
      "type": "category",
      "label": "Module 4 Vla",
      "items": [
        {
          "type": "doc",
          "id": "module-4-vla/index"
        },
        {
          "type": "doc",
          "id": "module-4-vla/chapter-4"
        },
        {
          "type": "doc",
          "id": "module-4-vla/chapter-3"
        },
        {
          "type": "doc",
          "id": "module-4-vla/chapter-2"
        },
        {
          "type": "doc",
          "id": "module-4-vla/chapter-1"
        }
      ]
    },
    {
      "type": "category",
      "label": "Module 3 Isaac",
      "items": [
        {
          "type": "doc",
          "id": "module-3-isaac/index"
        },
        {
          "type": "doc",
          "id": "module-3-isaac/chapter-4"
        },
        {
          "type": "doc",
          "id": "module-3-isaac/chapter-3"
        },
        {
          "type": "doc",
          "id": "module-3-isaac/chapter-2"
        },
        {
          "type": "doc",
          "id": "module-3-isaac/chapter-1"
        }
      ]
    },
    {
      "type": "category",
      "label": "Module 2 Digital Twin",
      "items": [
        {
          "type": "doc",
          "id": "module-2-digital-twin/index"
        },
        {
          "type": "doc",
          "id": "module-2-digital-twin/chapter-4"
        },
        {
          "type": "doc",
          "id": "module-2-digital-twin/chapter-3"
        },
        {
          "type": "doc",
          "id": "module-2-digital-twin/chapter-2"
        },
        {
          "type": "doc",
          "id": "module-2-digital-twin/chapter-1"
        }
      ]
    },
    {
      "type": "category",
      "label": "Module 1 Ros2",
      "items": [
        {
          "type": "doc",
          "id": "module-1-ros2/index"
        },
        {
          "type": "doc",
          "id": "module-1-ros2/chapter-4"
        },
        {
          "type": "doc",
          "id": "module-1-ros2/chapter-3"
        },
        {
          "type": "doc",
          "id": "module-1-ros2/chapter-2"
        },
        {
          "type": "doc",
          "id": "module-1-ros2/chapter-1"
        }
      ]
    }
  ]
};

export default sidebars;
