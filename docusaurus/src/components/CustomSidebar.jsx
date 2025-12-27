import React from 'react';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';
import { useDocsSidebar } from '@docusaurus/theme-common/internal';
import clsx from 'clsx';

import styles from './CustomSidebar.module.css';

/**
 * CustomSidebar Component
 * Implements the SidebarProps interface as defined in the contracts
 */
const CustomSidebar = (props) => {
  const location = useLocation();
  const sidebarData = useDocsSidebar();
  
  const { className } = props;

  // Define the exact sidebar order as required
  const sidebarOrder = [
    { id: 'intro', label: 'Introduction', href: '/docs/intro' },
    { id: 'module1-page1', label: 'Module 1', href: '/docs/module1/page1' },
    { id: 'module1-page2', label: 'Module 1 (cont.)', href: '/docs/module1/page2' },
    { id: 'module2-page1', label: 'Module 2', href: '/docs/module2/page1' },
    { id: 'module3-page1', label: 'Module 3', href: '/docs/module3/page1' },
    { id: 'module4-page1', label: 'Module 4', href: '/docs/module4/page1' },
  ];

  // Get current active item based on current location
  const activeItem = sidebarOrder.find(item => location.pathname.includes(item.href))?.id || '';

  return (
    <nav className={clsx('menu', styles.sidebar, className)}>
      <ul className="menu__list">
        {sidebarOrder.map((item, index) => (
          <li key={index} className="menu__list-item">
            <Link
              className={clsx(
                'menu__link',
                item.id === activeItem && 'menu__link--active'
              )}
              to={item.href}
              aria-current={item.id === activeItem ? 'page' : undefined}
            >
              {item.label}
            </Link>
          </li>
        ))}
      </ul>
    </nav>
  );
};

export default CustomSidebar;