import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: '— Steve Jobs',
    Img: require('@site/static/img/ci1.png').default,
    description: (
      <>
        Innovation distinguishes between a leader and a follower.
      </>
    ),
  },
  {
    title: '— Henry Ford',
    Img: require('@site/static/img/ci2.png').default,
    description: (
      <>
        Coming together is a beginning, staying together is progress, and working together is success.
      </>
    ),
  },
  {
    title: '— Alan Kay',
    Img: require('@site/static/img/ci3.png').default,
    description: (
      <>
        The best way to predict the future is to invent it.
      </>
    ),
  },
];

function Feature({ Img, title, description }) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <img className={styles.featureSvg} src={Img} alt={title} />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
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
