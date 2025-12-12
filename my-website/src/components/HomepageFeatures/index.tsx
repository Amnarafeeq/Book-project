import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  icon: string; // Using string emojis as icons instead of SVG files
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Comprehensive Curriculum',
    icon: '📚',
    description: (
      <>
        A complete learning path from foundational concepts to advanced implementations,
        with hands-on projects and real-world applications.
      </>
    ),
  },
  {
    title: 'Industry Standard Tools',
    icon: '🛠️',
    description: (
      <>
        Learn with cutting-edge technologies and frameworks used in professional
        robotics and AI development environments.
      </>
    ),
  },
  {
    title: 'Expert Guidance',
    icon: '👨‍🏫',
    description: (
      <>
        Access to professional resources, best practices, and mentorship from
        industry experts and researchers.
      </>
    ),
  },
];

function Feature({title, icon, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <div className={styles.featureIcon}>{icon}</div>
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
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
