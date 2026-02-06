import type { ReactNode } from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import Heading from "@theme/Heading";

import styles from "./index.module.css";

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx("hero hero--primary", styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/module-1-ros2/intro-physical-ai"
          >
            Start Learning
          </Link>
        </div>
      </div>
    </header>
  );
}

function Feature({
  title,
  description,
  link,
}: {
  title: string;
  description: string;
  link: string;
}) {
  return (
    <div className="col col--3">
      <div className="card margin--md padding--md">
        <div className="card__header">
          <h3>{title}</h3>
        </div>
        <div className="card__body">
          <p>{description}</p>
        </div>
        <div className="card__footer">
          <Link className="button button--primary button--block" to={link}>
            Explore
          </Link>
        </div>
      </div>
    </div>
  );
}

function HomepageFeatures() {
  return (
    <section className="margin-vert--lg">
      <div className="container">
        <div className="row">
          <Feature
            title="Module 1: ROS 2"
            description="Master the Robot Operating System 2, the foundation of modern robotics development."
            link="/module-1-ros2/intro-physical-ai"
          />
          <Feature
            title="Module 2: Simulation"
            description="Learn to build and test robots in Gazebo and Unity simulation environments."
            link="/module-2-simulation/gazebo-setup"
          />
          <Feature
            title="Module 3: NVIDIA Isaac"
            description="Leverage GPU-accelerated perception and simulation with NVIDIA Isaac platform."
            link="/module-3-isaac/isaac-overview"
          />
          <Feature
            title="Module 4: VLA Systems"
            description="Build Vision-Language-Action systems for intelligent humanoid robots."
            link="/module-4-vla/kinematics-dynamics"
          />
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="A comprehensive textbook for learning Physical AI and Humanoid Robotics"
    >
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
