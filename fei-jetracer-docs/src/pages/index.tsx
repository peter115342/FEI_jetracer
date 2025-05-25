import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import Translate, {translate} from '@docusaurus/Translate';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            <Translate id="homepage.getStarted">
              Get started 🏎️
            </Translate>
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description={translate({
        id: 'homepage.description',
        message: 'Description will go into a meta tag in <head />',
      })}>
      <HomepageHeader />
      <main>
        <div className={styles.featuresWrapper}>
          <HomepageFeatures />
        </div>
        <div className="container">
          <div className={styles.donkeyCarSection}>
            <div className={styles.donkeyCarInfo}>
              <Heading as="h3">
                <Translate id="homepage.feicarTitle">
                  FEIcar - DonkeyCar with Object Detection and different UI
                </Translate>
              </Heading>
              <p>
                <Translate id="homepage.feicarDescription">
                  We've created a fork of DonkeyCar called
                </Translate>{' '}
                <strong>FEIcar</strong>{' '}
                <Translate id="homepage.feicarDescription2">
                  that adds YOLOv4-tiny object detection capabilities to the standard DonkeyCar platform and reworked UI.
                </Translate>
              </p>
              <div className={styles.donkeyButtons}>
                <a
                  className="button button--outline button--primary button--md"
                  href="https://github.com/peter115342/FEIcar"
                  target="_blank"
                  rel="noopener noreferrer">
                  <Translate id="homepage.feicarRepo">
                    FEIcar GitHub Repository
                  </Translate>
                </a>
                <a
                  className="button button--outline button--secondary button--md"
                  href="https://github.com/autorope/donkeycar"
                  target="_blank"
                  rel="noopener noreferrer">
                  <Translate id="homepage.donkeycarRepo">
                    DonkeyCar GitHub Repository
                  </Translate>
                </a>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
