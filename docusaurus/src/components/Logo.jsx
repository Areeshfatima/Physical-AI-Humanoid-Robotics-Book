import React from 'react';
import clsx from 'clsx';
import styles from './Logo.module.css';

/**
 * Custom Logo Component
 * Implements the LogoProps interface as defined in the contracts
 */
const Logo = (props) => {
  const { src, alt, className, width, height } = props;

  return (
    <img
      src={src}
      alt={alt}
      className={clsx('navbar__logo', className)}
      width={width}
      height={height}
      style={{
        maxWidth: '100%',
        height: 'auto',
      }}
    />
  );
};

export default Logo;