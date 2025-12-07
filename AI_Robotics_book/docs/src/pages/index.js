import React, {useState, useEffect} from 'react';
import clsx from 'clsx';
import styles from './index.module.css';

export default function Home() {
  const [isOpen, setIsOpen] = useState(false);
  const [name, setName] = useState('');
  const [email, setEmail] = useState('');
  const [message, setMessage] = useState('');

  useEffect(() => {
    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          entry.target.classList.add(styles.reveal);
          observer.unobserve(entry.target);
        }
      });
    }, {threshold: 0.12});

    document.querySelectorAll('.revealOnScroll').forEach(el => observer.observe(el));

    return () => observer.disconnect();
  }, []);
  return (
    <main>
      {/* Hero Section */}
      <section className={clsx(styles.hero, 'revealOnScroll')}>
        <div className={styles.heroContent}>
          <h1 className={styles.title}>Physical AI & Humanoid Robotics</h1>
          <p className={styles.subtitle}>
            Bridging the gap between the digital brain and the physical body
          </p>
          <div className={styles.heroButtons}>
            <a href="/AI-driven-development/docs/01-intro-to-ros2" className={clsx(styles.button, styles.buttonPrimary)}>
              Start Learning →
            </a>
            <a href="https://github.com/MUKARRAM-ONE/AI-driven-development/tree/main/AI_Robotics_book" className={clsx(styles.button, styles.buttonSecondary)}>
              View on GitHub
            </a>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section className={clsx(styles.features, 'revealOnScroll')}>
        <h2>What You'll Learn</h2>
        <div className={styles.featureGrid}>
          <div className={clsx(styles.featureCard, 'revealOnScroll')}>
            <div className={styles.featureIcon}>🤖</div>
            <h3>ROS 2 Fundamentals</h3>
            <p>Master the Robot Operating System with hands-on examples and real-world applications.</p>
          </div>
          <div className={clsx(styles.featureCard, 'revealOnScroll')}>
            <div className={styles.featureIcon}>🎮</div>
            <h3>Digital Twins</h3>
            <p>Build virtual representations of robots using Gazebo and Isaac Sim.</p>
          </div>
          <div className={clsx(styles.featureCard, 'revealOnScroll')}>
            <div className={styles.featureIcon}>🧠</div>
            <h3>AI Integration</h3>
            <p>Implement intelligent behaviors using machine learning and planning algorithms.</p>
          </div>
          <div className={clsx(styles.featureCard, 'revealOnScroll')}>
            <div className={styles.featureIcon}>👁️</div>
            <h3>Vision & Language</h3>
            <p>Integrate VLMs and LLMs to enable robots to see, understand, and reason.</p>
          </div>
          <div className={clsx(styles.featureCard, 'revealOnScroll')}>
            <div className={styles.featureIcon}>📚</div>
            <h3>13 Chapters</h3>
            <p>Comprehensive curriculum from basics to advanced robotics concepts.</p>
          </div>
          <div className={clsx(styles.featureCard, 'revealOnScroll')}>
            <div className={styles.featureIcon}>💻</div>
            <h3>Hands-On Code</h3>
            <p>Learn by doing with complete code examples and interactive tutorials.</p>
          </div>
        </div>
      </section>

      {/* Modules Section */}
      <section className={clsx(styles.modules, 'revealOnScroll')}>
        <h2>Course Structure</h2>
        <div className={styles.moduleGrid}>
          <div className={clsx(styles.module, 'revealOnScroll')}>
            <h3>Module 1</h3>
            <h4>ROS 2 Fundamentals</h4>
            <ul>
              <li>Chapter 1: Introduction to ROS 2</li>
              <li>Chapter 2: Nodes, Topics & Services</li>
              <li>Chapter 3: Python Integration</li>
            </ul>
          </div>
          <div className={clsx(styles.module, 'revealOnScroll')}>
            <h3>Module 2</h3>
            <h4>Digital Twin & Simulation</h4>
            <ul>
              <li>Chapter 4: URDF for Humanoids</li>
              <li>Chapter 5: Gazebo Fundamentals</li>
              <li>Chapter 6: Unity Integration</li>
              <li>Chapter 7: Sensor Simulation</li>
              <li>Chapter 8: Isaac Sim Intro</li>
            </ul>
          </div>
          <div className={clsx(styles.module, 'revealOnScroll')}>
            <h3>Module 3</h3>
            <h4>AI-Robot Brain</h4>
            <ul>
              <li>Chapter 9: Isaac ROS vSLAM</li>
              <li>Chapter 10: Nav2 Path Planning</li>
            </ul>
          </div>
          <div className={clsx(styles.module, 'revealOnScroll')}>
            <h3>Module 4</h3>
            <h4>Vision Language Models</h4>
            <ul>
              <li>Chapter 11: VLM with Whisper</li>
              <li>Chapter 12: LLM Planning</li>
              <li>Chapter 13: Capstone Project</li>
            </ul>
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section className={clsx(styles.cta, 'revealOnScroll')}>
        <h2>Ready to Build Intelligent Robots?</h2>
        <p>Start with the fundamentals and progress to building your own autonomous systems.</p>
        <a href="/AI-driven-development/docs/01-intro-to-ros2" className={clsx(styles.button, styles.buttonLarge)}>
          Get Started Now
        </a>
      </section>

      {/* Contact Section */}
      <section className={clsx(styles.contact, 'revealOnScroll')}>
        <h2>Contact</h2>
        <p>If you'd like to get in touch, reach out via GitHub or LinkedIn — or send a quick message:</p>
        <div className={styles.contactButtons}>
          <a href="https://github.com/MUKARRAM-ONE/AI-driven-development" target="_blank" rel="noreferrer" className={clsx(styles.button, styles.buttonSecondary)}>
            Contact on GitHub
          </a>
          <a href="https://www.linkedin.com/in/your-linkedin" target="_blank" rel="noreferrer" className={clsx(styles.button, styles.buttonSecondary)}>
            Contact on LinkedIn
          </a>
          <button className={clsx(styles.button, styles.buttonPrimary)} onClick={() => setIsOpen(true)}>
            Send a message
          </button>
        </div>
      </section>

      {/* Contact Modal */}
      {isOpen && (
        <div className={styles.modalOverlay} onClick={() => setIsOpen(false)}>
          <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
            <h3>Send a message</h3>
            <label className={styles.label}>Name</label>
            <input className={styles.input} value={name} onChange={(e) => setName(e.target.value)} />
            <label className={styles.label}>Email</label>
            <input className={styles.input} value={email} onChange={(e) => setEmail(e.target.value)} />
            <label className={styles.label}>Message</label>
            <textarea className={styles.textarea} value={message} onChange={(e) => setMessage(e.target.value)} />
            <div style={{display: 'flex', gap: 12, marginTop: 12}}>
              <button className={clsx(styles.button, styles.buttonPrimary)} onClick={() => {
                const subject = encodeURIComponent('Contact from AI Robotics site');
                const body = encodeURIComponent(`Name: ${name}\nEmail: ${email}\n\n${message}`);
                window.location.href = `mailto:youremail@example.com?subject=${subject}&body=${body}`;
              }}>Send Email</button>
              <button className={clsx(styles.button)} onClick={() => setIsOpen(false)}>Close</button>
            </div>
          </div>
        </div>
      )}
    </main>
  );
}
