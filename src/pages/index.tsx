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
        <div className="row">
          <div className="col col--6">
            <div className="text--center padding-vert--md">
              <Heading as="h1" className="hero__title">
                {siteConfig.title}
              </Heading>
              <p className="hero__subtitle">{siteConfig.tagline}</p>
              <div className={styles.buttons}>
                <Link
                  className="button button--secondary button--lg"
                  to="/docs/intro">
                  Begin Your Learning Journey 🤖
                </Link>
              </div>
            </div>
          </div>
          <div className="col col--6 text--center padding-vert--md">
            <div className={styles.heroImageContainer}>
              <img
                src="/img/ai-brain-placeholder.svg"
                alt="Physical AI and Humanoid Robotics Concept"
                className={styles.heroImage}
                onError={(e) => {
                  const target = e.target as HTMLImageElement;
                  target.onerror = null; // Prevent infinite loop
                  target.src = "/img/robot-placeholder.svg";
                }}
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
      description="Comprehensive educational resource for Physical AI & Humanoid Robotics - Your complete textbook on the intersection of artificial intelligence and robotics">
      <HomepageHeader />
      <main>
        <section className={styles.textbookIntro}>
          <div className="container padding-vert--lg">
            <div className="row">
              <div className="col col--6 col--offset-3 padding-horiz--md">
                <div className="textbook-content">
                  <Heading as="h2" className="text--center padding-bottom--md">
                    About This Textbook
                  </Heading>
                  <p>
                    Welcome to the comprehensive textbook on <strong>Physical AI & Humanoid Robotics</strong>.
                    This educational resource provides an in-depth exploration of the intersection between artificial
                    intelligence and robotics, with a specific focus on humanoid robot systems.
                  </p>
                  <p>
                    Through this course, you will learn about:
                  </p>
                  <ul>
                    <li>The theoretical foundations of physical AI</li>
                    <li>Core robotics concepts including perception, planning, and control</li>
                    <li>Advanced topics in humanoid robot design and functionality</li>
                    <li>Real-world applications and case studies</li>
                    <li>Integration of vision-language-action models in robotic systems</li>
                  </ul>
                </div>
              </div>
            </div>
          </div>
        </section>
        <section className={styles.learningPath}>
          <div className="container padding-vert--lg">
            <div className="row">
              <div className="col col--10 col--offset-1">
                <Heading as="h2" className="text--center padding-bottom--lg">
                  Structured Learning Path
                </Heading>
                <div className="row">
                  <div className="col col--3">
                    <div className="textbook-content text--center">
                      <h3>Module 1</h3>
                      <p>ROS2 Fundamentals</p>
                    </div>
                  </div>
                  <div className="col col--3">
                    <div className="textbook-content text--center">
                      <h3>Module 2</h3>
                      <p>Digital Twin & Simulation</p>
                    </div>
                  </div>
                  <div className="col col--3">
                    <div className="textbook-content text--center">
                      <h3>Module 3</h3>
                      <p>Isaac Sim & Navigation</p>
                    </div>
                  </div>
                  <div className="col col--3">
                    <div className="textbook-content text--center">
                      <h3>Module 4</h3>
                      <p>Vision-Language-Action Systems</p>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
