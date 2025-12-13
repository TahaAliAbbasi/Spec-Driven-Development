import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import CardsGrid from '@site/src/components/CardsGrid';
import Heading from '@theme/Heading';

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
            Read Docs
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
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <div className="container">
          <br/>
          <Heading as="h1" className="hero__title">
            What's in this book ?
          </Heading>
          <p className="hero__subtitle">This book is a complete introduction to bringing AI into the real physical world through humanoid robotics. Instead of keeping AI limited to screens and software, the book teaches you how to build robots that can sense their environment, understand what is happening, and act intelligently. You will learn how AI can be connected to physical bodies, allowing robots to move, react, and interact with humans in natural ways.</p>

          <p className="hero__subtitle">The journey begins with ROS 2, the main operating system for robots. You will understand how a robot's “nervous system” works—how it sends messages, controls motors, reads sensors, and follows commands. After mastering the basics of robot control, the book takes you into simulation. Using Gazebo and Unity, you will create a digital twin of your robot, test movements, simulate physics, and build environments where your humanoid robot can safely practice and learn.</p>

          <p className="hero__subtitle">Next, the book explores NVIDIA Isaac, a powerful platform for training robots with AI. You will learn how robots can develop perception, navigation, and decision-making using advanced tools like Isaac Sim and Isaac ROS. These tools allow robots to understand the world visually, map their surroundings, and plan paths in a human-like way.</p>

          <p className="hero__subtitle">Finally, the book shows how modern AI models can be connected to robots to create natural interactions. You will learn how to use systems like GPT for conversation and Whisper for voice control. By the end of the book, you will build a complete simulated humanoid robot that can listen to your voice, think about what to do, navigate around obstacles, find objects, and perform tasks on its own. This book is an accessible but powerful guide to Physical AI—where digital intelligence meets real-world action through advanced humanoid robots.</p>
        <Heading as="h1" className="hero__title">
          Modules :-
        </Heading>
        <CardsGrid />
        </div>
      </main>
    </Layout>
  );
}
