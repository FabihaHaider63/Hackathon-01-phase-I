import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">Physical AI & Humanoid Robotics</h1>
        <p className="hero__subtitle">
          A comprehensive guide covering Physical AI and Humanoid Robotics from theory to full system implementation
        </p>
      </div>
    </header>
  );
}

function BookOverviewBox() {
  return (
    <div className={clsx('card', styles.card)}>
      <div className="card__body">
        <h2>Book Overview</h2>
        <p>
          This comprehensive textbook provides an in-depth exploration of Physical AI and Humanoid Robotics,
          bridging the gap between theoretical foundations and practical implementation. From kinematics and
          dynamics to control systems and machine learning integration, this guide offers everything needed
          to understand and develop advanced humanoid robots.
        </p>

        <h3>Key Modules:</h3>
        <ul>
          <li>Fundamentals of Robotics and Kinematics</li>
          <li>Humanoid Locomotion and Gait Control</li>
          <li>Sensor Integration and Perception Systems</li>
          <li>Machine Learning for Robot Control</li>
          <li>Human-Robot Interaction and Social Robotics</li>
          <li>Ethics and Safety in Physical AI</li>
        </ul>

        <div className={styles.metaInfo}>
          <p><strong>Audience:</strong> Students, Developers, Researchers</p>
          <p><strong>Difficulty:</strong> Beginner → Advanced</p>
        </div>
      </div>
    </div>
  );
}

function AuthorDetailsBox() {
  return (
    <div className={clsx('card', styles.card)}>
      <div className="card__body">
        <h2>Author Details</h2>
        <h3>Fabiha Haider</h3>
        <p>Developer</p>
        <p>
         I am  Fabiha Haider qwen a passionate developer and researcher in the field of Physical AI and Humanoid Robotics.
          With extensive experience in building intelligent robotic systems, she combines theoretical knowledge with
          practical implementation to create accessible educational content.
        </p>

        <h3>Skills:</h3>
        <p>HTML, CSS, TypeScript, Next.js, Python, OpenAI SDK, Prompt Engineering, Figma, Documentation</p>

        <div className={styles.socialLinks}>
          <h3>Connect whith Fabiha</h3>
          <ul>
            <li>
              <a href="https://github.com/FabihaHaider63" target="_blank" rel="noopener noreferrer">
                GitHub
              </a>
            </li>
            <li>
              <a
                href="https://www.linkedin.com/in/fabiha-haider-143117322?utm_source=share&utm_campaign=share_via&utm_content=profile&utm_medium=android_app"
                target="_blank"
                rel="noopener noreferrer"
              >
                LinkedIn
              </a>
            </li>
            <li>
              <a href="https://x.com/fabiha62336?t=nD1-fB6mth5Fr6gP6SZ4xw&s=09" target="_blank" rel="noopener noreferrer">
                X (Twitter)
              </a>
            </li>
            <li>
              <a href="mailto:fabihahaider633@gmail.com">Email</a>
            </li>
          </ul>
        </div>
      </div>
    </div>
  );
}

function FooterSection() {
  return (
    <footer className={styles.footer}>
      <div className="container">
        <h2 className={styles.footerHeading}>
          <span>Connect with me</span>
        </h2>
        <ul className={styles.footerLinks}>
          <li>
            <a href="https://fabiha-haider-63.vercel.app/" target="_blank" rel="noopener noreferrer">
              Portfolio
            </a>
          </li>
          <li>
            <a
              href="https://www.linkedin.com/in/fabiha-haider-143117322?utm_source=share&utm_campaign=share_via&utm_content=profile&utm_medium=android_app"
              target="_blank"
              rel="noopener noreferrer"
            >
              LinkedIn
            </a>
          </li>
          <li>
            <a href="https://x.com/fabiha62336?t=nD1-fB6mth5Fr6gP6SZ4xw&s=09" target="_blank" rel="noopener noreferrer">
              Twitter
            </a>
          </li>
          <li>
            <a href="https://github.com/FabihaHaider63" target="_blank" rel="noopener noreferrer">
              GitHub
            </a>
          </li>
          <li>
            <a href="mailto:fabihahaider633@gmail.com">Email</a>
          </li>
        </ul>
      </div>
    </footer>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />"
      footer={null}>  {/* Disable default footer */}
      <main>
        <div className={styles.mainContainer}>
          <HomepageHeader />
          <div className={styles.contentGrid}>
            <BookOverviewBox />
            <AuthorDetailsBox />
          </div>
          <FooterSection />
        </div>
      </main>
    </Layout>
  );
}