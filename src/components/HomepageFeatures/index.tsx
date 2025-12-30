import type { ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Easy to Use',
    description: (
      <>
        This book is designed to be simple and practical, helping you understand
        Physical AI and Humanoid Robotics step by step.
      </>
    ),
  },
  {
    title: 'Focus on Core Concepts',
    description: (
      <>
        Concentrate on AI reasoning, perception, control systems, and real-world
        robotic applications without unnecessary complexity.
      </>
    ),
  },
  {
    title: 'Powered by Modern AI',
    description: (
      <>
        Learn using modern AI tools, simulations, and real-world examples built
        with React, Python, and robotics frameworks.
      </>
    ),
  },
];

function Feature({ title, description }: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
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
