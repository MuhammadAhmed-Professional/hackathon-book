import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  const bookCoverUrl = useBaseUrl('/img/book-cover.png');
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className="hero__title">
              {siteConfig.title}
            </Heading>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Start Learning Physical AI
              </Link>
              <Link
                className="button button--outline button--lg"
                to="/docs/hardware"
                style={{marginLeft: '1rem'}}>
                Hardware Requirements
              </Link>
            </div>
          </div>
          <div className={styles.heroImage}>
            <img
              src={bookCoverUrl}
              alt="Physical AI & Humanoid Robotics Book Cover"
              className={styles.bookCover}
              onError={(e) => {
                console.error('Failed to load book cover image:', bookCoverUrl);
                e.target.style.display = 'none';
              }}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title} - 13-Week Course`}
      description="Comprehensive textbook covering Physical AI & Humanoid Robotics: ROS 2, Gazebo/Unity, NVIDIA Isaac, and Vision-Language-Action systems for building autonomous humanoid robots.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
