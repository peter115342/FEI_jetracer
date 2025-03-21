import React, { JSX } from 'react';
import clsx from 'clsx';
import Translate from '@docusaurus/Translate';
import styles from './styles.module.css';

type FeatureItem = {
  title: JSX.Element;
  description: JSX.Element;
};

const FeatureList: FeatureItem[] = [
  {
    title: <Translate>Easy to Assemble</Translate>,
    description: (
      <Translate>
        Follow the step-by-step assembly guide to build your JetRacer robot quickly and easily.
      </Translate>
    ),
  },
  {
    title: <Translate>Powered by ROS</Translate>,
    description: (
      <Translate>
        Built on the Robot Operating System (ROS), providing a flexible framework for robot software development.
      </Translate>
    ),
  },
  {
    title: <Translate>AI Capabilities</Translate>,
    description: (
      <Translate>
        Leverage the power of NVIDIA Jetson Nano to run AI models for computer vision and autonomous navigation.
      </Translate>
    ),
  },
  {
    title: <Translate>DonkeyCar Integration</Translate>,
    description: (
      <Translate>
        Compatible with DonkeyCar platform, allowing you to build and train autonomous vehicles using machine learning.
      </Translate>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
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
