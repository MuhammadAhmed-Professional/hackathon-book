import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Module 1: ROS 2 - The Robotic Nervous System',
    description: (
      <>
        Master ROS 2 architecture, nodes, topics, services, and actions. Build Python-based
        ROS 2 packages and define humanoid robot structures using URDF. Learn the foundational
        software framework powering modern robotics.
      </>
    ),
  },
  {
    title: 'Module 2: Gazebo & Unity - The Digital Twin',
    description: (
      <>
        Simulate robot behaviors in Gazebo physics engine and Unity for photorealistic rendering.
        Test sensors (LiDAR, depth cameras, IMUs) and physics (gravity, collisions) safely
        before real-world deployment.
      </>
    ),
  },
  {
    title: 'Module 3: NVIDIA Isaac - The AI-Robot Brain',
    description: (
      <>
        Explore Isaac Sim for synthetic data generation and Isaac ROS for hardware-accelerated
        perception (VSLAM, object detection). Implement autonomous navigation with Nav2 for
        bipedal humanoid robots.
      </>
    ),
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    description: (
      <>
        Integrate voice commands (OpenAI Whisper), cognitive planning (LLMs), and robot actions
        into autonomous systems. Complete the capstone: a humanoid that understands natural
        language and navigates to accomplish tasks.
      </>
    ),
  },
  {
    title: 'Interactive RAG Chatbot',
    description: (
      <>
        Ask questions about any chapter and receive AI-powered answers grounded in textbook
        content. Select specific code examples or technical sections for targeted explanations.
        Powered by OpenAI and Qdrant vector search.
      </>
    ),
  },
  {
    title: 'Hardware Requirements & Alternatives',
    description: (
      <>
        Detailed specifications for workstations (RTX 4070 Ti+, Ubuntu 22.04), Jetson Orin kits,
        RealSense cameras, and robot platforms (Unitree Go2/G1). Cloud-based alternatives for
        students without access to high-end hardware.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
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
