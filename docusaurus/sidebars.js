// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1',
      items: ['module1/page1', 'module1/page2'],
    },
    {
      type: 'category',
      label: 'Module 2',
      items: ['module2/page1', 'module2/page2'],
    },
    {
      type: 'category',
      label: 'Module 3',
      items: ['module3/page1', 'module3/page2'],
    },
    {
      type: 'category',
      label: 'Module 4',
      items: ['module4/page1', 'module4/page2'],
    },
  ],
};

module.exports = sidebars;