import React from 'react';
import clsx from 'clsx';
import styles from './HeroSection.module.css';

/**
 * Custom HeroSection Component
 * Implements the HeroSectionProps interface as defined in the contracts
 */
const HeroSection = (props) => {
  const { title, subtitle, images, ctaButton, className } = props;

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner, className)}>
      <div className="container">
        <h1 className="hero__title">{title}</h1>
        <p className="hero__subtitle">{subtitle}</p>
        <div className="hero__images">
          {images && images.map((image, index) => (
            <div key={index} className={styles.imageContainer}>
              <img 
                src={image.src} 
                alt={image.alt} 
                title={image.caption}
                className={styles.heroImage}
              />
            </div>
          ))}
        </div>
        {ctaButton && (
          <div className={styles.buttonContainer}>
            <a
              className={clsx(
                'button button--secondary button--lg',
                styles.heroButton
              )}
              href={ctaButton.href}
            >
              {ctaButton.label}
            </a>
          </div>
        )}
      </div>
    </header>
  );
};

export default HeroSection;