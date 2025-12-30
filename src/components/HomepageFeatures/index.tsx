import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI Fundamentals',
    Svg: require('@site/static/img/001-docusaurus-textbook/module-placeholder.svg').default,
    description: (
      <>
        Understand the core principles of Physical AI, where artificial intelligence meets real-world
        interaction through robotic systems and sensors.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics Design',
    Svg: require('@site/static/img/003-digital-twin-gazebo-unity/module-placeholder.svg').default,
    description: (
      <>
        Learn about the design and engineering challenges in creating humanoid robots with human-like
        structure and capabilities.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action Systems',
    Svg: require('@site/static/img/005-vla-systems/module-placeholder.svg').default,
    description: (
      <>
        Explore next-generation AI systems that integrate visual perception, language understanding,
        and physical action to create intelligent robots.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container padding-vert--lg">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
