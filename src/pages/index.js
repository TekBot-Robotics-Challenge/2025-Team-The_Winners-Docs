import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div
          className={styles.buttons}
          style={{
            display: 'flex',
            justifyContent: 'center',
            gap: '1rem',          // espace entre les images
            flexWrap: 'wrap'      // passe à la ligne si trop petit
          }}
        >
          <img
            src="/img/vid.gif"
            alt="Team working"
            style={{ width: '30%', borderRadius: '10px' }}
          />
          <img
            src="/img/vid.gif"
            alt="Robot presentation"
            style={{ width: '30%', borderRadius: '10px' }}
          />
          <img
            src="/img/vid.gif"
            alt="Team testing"
            style={{ width: '30%', borderRadius: '10px' }}
          />
        </div>
      </div>
    </header>
  );
}



export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`The Winners ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
