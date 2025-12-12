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
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.bookCover}>
          <div className={styles.bookSpine}>
            <div className={styles.spineLogo}>
              <img
                src="/img/logo.svg"
                alt="Robotics Logo"
                className={styles.logoImage}
              />
            </div>
            <div className={styles.spineTitle}>
              <div className={styles.spineBookTitle}>{siteConfig.title}</div>
            </div>
          </div>
          <div className={styles.bookContent}>
            <div className={styles.bookHeader}>
              <div className={styles.bookBadge}>TEXTBOOK</div>
              <Heading as="h1" className={styles.bookTitle}>
                {siteConfig.title}
              </Heading>
              <p className={styles.bookSubtitle}>{siteConfig.tagline}</p>
            </div>

            <div className={styles.bookCoverImage}>
              <img
                src="/img/robot-illustration-placeholder.png"
                alt="Physical AI & Robotics Concept"
                className={styles.coverImage}
              />
            </div>

            <div className={styles.bookDetails}>
              <div className={styles.authorInfo}>
                <p className={styles.author}>Physical AI & Robotics Academy</p>
                <p className={styles.edition}>First Edition</p>
              </div>
              <div className={styles.bookActions}>
                <Link
                  className="button button--primary button--lg"
                  to="/docs/overview">
                  Start Reading
                </Link>
                <Link
                  className="button button--secondary button--lg"
                  to="/docs/modules">
                  Explore Modules
                </Link>
                <div className={styles.moduleButtons}>
                  <Link
                    className="button button--outline button--sm"
                    to="/docs/modules/module-1-ros2">
                    Module 1: ROS 2
                  </Link>
                  <Link
                    className="button button--outline button--sm"
                    to="/docs/modules/module-2-gazebo-unity">
                    Module 2: Gazebo & Unity
                  </Link>
                  <Link
                    className="button button--outline button--sm"
                    to="/docs/modules/module-3-nvidia-isaac">
                    Module 3: NVIDIA Isaac
                  </Link>
                  <Link
                    className="button button--outline button--sm"
                    to="/docs/modules/module-4-vla">
                    Module 4: VLA Systems
                  </Link>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function WhatYoullLearnSection() {
  return (
    <section className={styles.whatYoullLearnSection}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={styles.sectionTitle}>
              What You'll Learn
            </Heading>
            <div className={styles.learningGrid}>
              <div className={styles.learningItem}>
                <div className={styles.learningIcon}>🤖</div>
                <h3 className={styles.learningTitle}>Physical Intelligence</h3>
                <p className={styles.learningDescription}>
                  Understand how AI systems can perceive, reason, and act in the physical world,
                  bridging the gap between digital intelligence and embodied systems.
                </p>
              </div>
              <div className={styles.learningItem}>
                <div className={styles.learningIcon}>🦾</div>
                <h3 className={styles.learningTitle}>Humanoid Robotics</h3>
                <p className={styles.learningDescription}>
                  Explore the design, control, and implementation of humanoid robots with
                  advanced locomotion, manipulation, and interaction capabilities.
                </p>
              </div>
              <div className={styles.learningItem}>
                <div className={styles.learningIcon}>🧠</div>
                <h3 className={styles.learningTitle}>Embodied AI Systems</h3>
                <p className={styles.learningDescription}>
                  Learn to build intelligent systems that learn through environmental interaction,
                  combining perception, planning, and control for real-world applications.
                </p>
              </div>
              <div className={styles.learningItem}>
                <div className={styles.learningIcon}>🔬</div>
                <h3 className={styles.learningTitle}>Advanced Control Systems</h3>
                <p className={styles.learningDescription}>
                  Master the control algorithms and systems that enable robots to move, balance,
                  and interact with their environment effectively.
                </p>
              </div>
              <div className={styles.learningItem}>
                <div className={styles.learningIcon}>🌐</div>
                <h3 className={styles.learningTitle}>Multi-Robot Systems</h3>
                <p className={styles.learningDescription}>
                  Understand coordination and collaboration between multiple robotic agents
                  in complex, dynamic environments.
                </p>
              </div>
              <div className={styles.learningItem}>
                <div className={styles.learningIcon}>🛡️</div>
                <h3 className={styles.learningTitle}>Safety & Ethics</h3>
                <p className={styles.learningDescription}>
                  Learn about safety protocols, ethical considerations, and responsible development
                  of AI and robotic systems.
                </p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function BookContentSection() {
  return (
    <section className={styles.bookContentSection}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={styles.sectionTitle}>
              Book Content Overview
            </Heading>
            <div className={styles.contentGrid}>
              <div className={styles.contentItem}>
                <h3 className={styles.contentTitle}>Part I: Foundations</h3>
                <ul className={styles.contentList}>
                  <li>Introduction to Physical AI</li>
                  <li>Robotics Fundamentals</li>
                  <li>Sensor Integration & Perception</li>
                  <li>Kinematics & Dynamics</li>
                </ul>
              </div>
              <div className={styles.contentItem}>
                <h3 className={styles.contentTitle}>Part II: Core Systems</h3>
                <ul className={styles.contentList}>
                  <li>Control Systems & Algorithms</li>
                  <li>Locomotion & Mobility</li>
                  <li>Manipulation & Grasping</li>
                  <li>Human-Robot Interaction</li>
                </ul>
              </div>
              <div className={styles.contentItem}>
                <h3 className={styles.contentTitle}>Part III: Advanced Topics</h3>
                <ul className={styles.contentList}>
                  <li>Learning & Adaptation</li>
                  <li>Multi-Robot Coordination</li>
                  <li>Safety & Ethics</li>
                  <li>Future Directions</li>
                </ul>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function TechnologyStackSection() {
  return (
    <section className={styles.techStackSection}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={styles.sectionTitle}>
              Technology Stack
            </Heading>
            <div className={styles.techGrid}>
              <div className={styles.techItem}>
                <div className={styles.techLogo}>ROS2</div>
                <p className={styles.techDescription}>Robot Operating System 2</p>
              </div>
              <div className={styles.techItem}>
                <div className={styles.techLogo}>Gazebo</div>
                <p className={styles.techDescription}>Robot Simulation</p>
              </div>
              <div className={styles.techItem}>
                <div className={styles.techLogo}>Python</div>
                <p className={styles.techDescription}>Primary Programming Language</p>
              </div>
              <div className={styles.techItem}>
                <div className={styles.techLogo}>C++</div>
                <p className={styles.techDescription}>Performance-Critical Systems</p>
              </div>
              <div className={styles.techItem}>
                <div className={styles.techLogo}>PyTorch</div>
                <p className={styles.techDescription}>Machine Learning Framework</p>
              </div>
              <div className={styles.techItem}>
                <div className={styles.techLogo}>OpenCV</div>
                <p className={styles.techDescription}>Computer Vision</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics - Textbook`}
      description="Complete textbook on Physical AI & Humanoid Robotics - From Digital Intelligence to Embodied Systems">
      <HomepageHeader />
      <main>
        <section className={styles.introductionSection}>
          <div className="container">
            <div className="row">
              <div className="col col--12">
                <Heading as="h2" className={styles.sectionTitle}>
                  About This Textbook
                </Heading>
                <p className={styles.sectionDescription}>
                  This comprehensive textbook covers the cutting-edge field of Physical AI and Humanoid Robotics,
                  bridging the gap between digital intelligence and embodied systems. From foundational concepts
                  to advanced implementations, this resource provides everything needed to understand and build
                  intelligent robotic systems.
                </p>
              </div>
            </div>
          </div>
        </section>

        <WhatYoullLearnSection />
        <BookContentSection />
        <TechnologyStackSection />

        <HomepageFeatures />
      </main>
    </Layout>
  );
}
