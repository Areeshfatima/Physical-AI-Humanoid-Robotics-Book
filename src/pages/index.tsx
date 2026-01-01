import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="textbook-cover">
          <div className={styles.textbookContent}>
            <Heading as="h1" className={styles.heroTitle}>
              Physical AI & Humanoid Robotics
            </Heading>
            <p className={styles.heroSubtitle}>An Academic Textbook</p>

            {/* Academic Abstract-style description */}
            <div className={styles.techDescription}>
              <p>
                Embodied intelligence systems that bridge the gap between artificial intelligence and physical interaction. This textbook explores the foundational principles, methodologies, and applications of humanoid robotics.
              </p>

              <p>
                <strong>Key Topics Include:</strong>
              </p>
              <ul>
                <li>Perception-action loops with multimodal sensor fusion</li>
                <li>Learning-based control strategies for dynamic environments</li>
                <li>Simulation-to-real transfer techniques and digital twin methodologies</li>
                <li>Human-scale robotic cognition and interaction paradigms</li>
              </ul>
            </div>

            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/introduction">
                Read Textbook
              </Link>
            </div>
          </div>

          <div className={styles.heroGraphics}>
            {/* Rotating/stacked hero diagrams */}
            <div className={styles.heroGraphic}>
              <img
                src="/img/hero/physical-ai-system.svg"
                alt="Physical AI Architecture"
                className={styles.heroImg}
              />
            </div>
            <div className={styles.heroGraphic}>
              <img
                src="/img/hero/humanoid-embodiment.svg"
                alt="Humanoid Embodiment & Sensors"
                className={styles.heroImg}
              />
            </div>
            <div className={styles.heroGraphic}>
              <img
                src="/img/hero/simulation-loop.svg"
                alt="Simulation to Reality Loop"
                className={styles.heroImg}
              />
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics Textbook`}
      description="Comprehensive academic textbook for Physical AI & Humanoid Robotics - Advanced educational resource on embodied intelligence and humanoid robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
