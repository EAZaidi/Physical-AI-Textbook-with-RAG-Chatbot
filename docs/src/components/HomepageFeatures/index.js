import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Hands-On Learning',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Build real humanoid robots from scratch with step-by-step tutorials,
        code examples, and practical exercises covering URDF modeling, ROS 2,
        and simulation.
      </>
    ),
  },
  {
    title: 'Industry-Standard Tools',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Master Gazebo for physics simulation, Unity for visualization, and
        ROS 2 for robot control—the same tools used by leading robotics companies.
      </>
    ),
  },
  {
    title: 'Complete Curriculum',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Progress from basic robot modeling to advanced perception and control,
        with modules covering digital twins, sensor simulation, and autonomous navigation.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
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
