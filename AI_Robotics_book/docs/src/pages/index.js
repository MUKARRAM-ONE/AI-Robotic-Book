import React, {useState, useEffect} from 'react';

export default function Home() {
  const [isOpen, setIsOpen] = useState(false);
  const [name, setName] = useState('');
  const [email, setEmail] = useState('');
  const [message, setMessage] = useState('');

  // Handle GitHub Pages 404 redirect on client side
  useEffect(() => {
    // Check for redirect parameter (from 404.html)
    const params = new URLSearchParams(window.location.search);
    const redirect = params.get('redirect');
    
    if (redirect) {
      // Navigate to the redirect path
      window.location.href = redirect;
      return;
    }
  }, []);

  useEffect(() => {
    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          entry.target.classList.add('reveal-animation');
          observer.unobserve(entry.target);
        }
      });
    }, {threshold: 0.12});

    document.querySelectorAll('.scroll-reveal').forEach(el => observer.observe(el));

    return () => observer.disconnect();
  }, []);

  const styles = {
    hero: {
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      minHeight: '100vh',
      background: 'linear-gradient(135deg, #1e3a8a 0%, #7c3aed 50%, #ec4899 100%)',
      color: 'white',
      padding: '60px 20px 40px',
      textAlign: 'center',
    },
    heroContent: {
      maxWidth: '900px',
      margin: '0 auto',
    },
    title: {
      fontSize: 'clamp(3rem, 8vw, 5.5rem)',
      fontWeight: '900',
      margin: '0 0 24px 0',
      lineHeight: '1.1',
      textShadow: '0 4px 20px rgba(0,0,0,0.3)',
    },
    subtitle: {
      fontSize: 'clamp(1.2rem, 3vw, 1.8rem)',
      margin: '0 0 40px 0',
      opacity: '0.95',
      lineHeight: '1.6',
    },
    heroButtons: {
      display: 'flex',
      gap: '20px',
      justifyContent: 'center',
      flexWrap: 'wrap',
    },
    button: {
      padding: '16px 36px',
      borderRadius: '12px',
      fontSize: '1.1rem',
      fontWeight: '700',
      textDecoration: 'none',
      transition: 'all 0.3s ease',
      display: 'inline-flex',
      alignItems: 'center',
      gap: '8px',
      border: 'none',
      cursor: 'pointer',
    },
    buttonPrimary: {
      background: 'white',
      color: '#1e3a8a',
      boxShadow: '0 8px 24px rgba(255,255,255,0.2)',
    },
    buttonSecondary: {
      background: 'rgba(255, 255, 255, 0.1)',
      color: 'white',
      border: '2px solid white',
      backdropFilter: 'blur(10px)',
    },
    features: {
      padding: '100px 20px',
      background: '#0f172a',
    },
    featuresTitle: {
      textAlign: 'center',
      fontSize: 'clamp(2.5rem, 5vw, 3.5rem)',
      marginBottom: '20px',
      color: 'white',
      fontWeight: '900',
    },
    featuresSubtitle: {
      textAlign: 'center',
      fontSize: '1.2rem',
      color: '#94a3b8',
      marginBottom: '60px',
      maxWidth: '700px',
      margin: '0 auto 60px',
    },
    featureGrid: {
      display: 'grid',
      gridTemplateColumns: 'repeat(auto-fit, minmax(300px, 1fr))',
      gap: '30px',
      maxWidth: '1400px',
      margin: '0 auto',
    },
    featureCard: {
      background: 'rgba(30, 41, 59, 0.8)',
      padding: '40px 30px',
      borderRadius: '20px',
      textAlign: 'center',
      border: '1px solid rgba(148, 163, 184, 0.2)',
      backdropFilter: 'blur(10px)',
      transition: 'all 0.4s ease',
    },
    featureIcon: {
      fontSize: '4rem',
      marginBottom: '20px',
    },
    featureTitle: {
      fontSize: '1.6rem',
      marginBottom: '15px',
      color: 'white',
      fontWeight: '700',
    },
    featureDesc: {
      color: '#cbd5e1',
      lineHeight: '1.7',
      margin: '0',
      fontSize: '1.05rem',
    },
    modules: {
      padding: '100px 20px',
      background: '#1e293b',
    },
    moduleGrid: {
      display: 'grid',
      gridTemplateColumns: 'repeat(auto-fit, minmax(280px, 1fr))',
      gap: '30px',
      maxWidth: '1400px',
      margin: '0 auto',
    },
    module: {
      background: 'linear-gradient(135deg, #3b82f6 0%, #8b5cf6 50%, #ec4899 100%)',
      backgroundSize: '200% 200%',
      color: 'white',
      padding: '40px 30px',
      borderRadius: '20px',
      boxShadow: '0 10px 40px rgba(0, 0, 0, 0.3)',
      transition: 'all 0.3s ease',
    },
    moduleNumber: {
      fontSize: '0.9rem',
      opacity: '0.9',
      margin: '0 0 10px 0',
      fontWeight: '700',
      letterSpacing: '1px',
    },
    moduleTitle: {
      fontSize: '1.8rem',
      margin: '0 0 25px 0',
      fontWeight: '800',
    },
    moduleList: {
      listStyle: 'none',
      padding: '0',
      margin: '0',
    },
    moduleItem: {
      padding: '10px 0',
      lineHeight: '1.6',
      opacity: '0.95',
      fontSize: '1.05rem',
    },
    cta: {
      padding: '120px 20px',
      textAlign: 'center',
      background: 'linear-gradient(135deg, #1e3a8a 0%, #7c3aed 50%, #ec4899 100%)',
      color: 'white',
    },
    ctaTitle: {
      fontSize: 'clamp(2.5rem, 5vw, 3.5rem)',
      marginBottom: '20px',
      fontWeight: '900',
    },
    ctaText: {
      fontSize: '1.3rem',
      marginBottom: '50px',
      opacity: '0.95',
      maxWidth: '700px',
      margin: '0 auto 50px',
      lineHeight: '1.6',
    },
    contact: {
      padding: '80px 20px',
      background: '#0f172a',
      textAlign: 'center',
    },
    contactTitle: {
      fontSize: '2.5rem',
      marginBottom: '16px',
      color: 'white',
      fontWeight: '900',
    },
    contactText: {
      color: '#94a3b8',
      fontSize: '1.2rem',
      marginBottom: '40px',
    },
    contactButtons: {
      display: 'flex',
      gap: '16px',
      justifyContent: 'center',
      marginTop: '16px',
      flexWrap: 'wrap',
    },
    modalOverlay: {
      position: 'fixed',
      inset: '0',
      background: 'rgba(0,0,0,0.8)',
      backdropFilter: 'blur(8px)',
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      zIndex: '1000',
    },
    modal: {
      background: '#1e293b',
      padding: '40px',
      borderRadius: '20px',
      width: '90%',
      maxWidth: '520px',
      boxShadow: '0 20px 60px rgba(0,0,0,0.5)',
      border: '1px solid rgba(148, 163, 184, 0.2)',
    },
    modalTitle: {
      color: 'white',
      fontSize: '2rem',
      marginBottom: '30px',
      fontWeight: '800',
    },
    label: {
      display: 'block',
      marginTop: '20px',
      fontWeight: '600',
      color: '#e2e8f0',
      marginBottom: '8px',
    },
    input: {
      width: '100%',
      padding: '14px 16px',
      borderRadius: '10px',
      border: '1px solid rgba(148, 163, 184, 0.3)',
      background: '#0f172a',
      color: 'white',
      fontSize: '1rem',
      boxSizing: 'border-box',
    },
    textarea: {
      width: '100%',
      minHeight: '140px',
      padding: '14px 16px',
      borderRadius: '10px',
      border: '1px solid rgba(148, 163, 184, 0.3)',
      background: '#0f172a',
      color: 'white',
      fontSize: '1rem',
      resize: 'vertical',
      boxSizing: 'border-box',
    },
    chatButton: {
      position: 'fixed',
      bottom: '30px',
      right: '30px',
      width: '70px',
      height: '70px',
      borderRadius: '50%',
      background: 'linear-gradient(135deg, #3b82f6 0%, #8b5cf6 100%)',
      border: 'none',
      boxShadow: '0 8px 30px rgba(59, 130, 246, 0.6)',
      cursor: 'pointer',
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      color: 'white',
      fontSize: '2rem',
      zIndex: '999',
      transition: 'all 0.3s ease',
    },
    chatWidget: {
      position: 'fixed',
      bottom: '30px',
      right: '30px',
      width: '400px',
      height: '550px',
      background: '#1e293b',
      borderRadius: '20px',
      boxShadow: '0 20px 60px rgba(0,0,0,0.5)',
      border: '1px solid rgba(148, 163, 184, 0.2)',
      display: 'flex',
      flexDirection: 'column',
      zIndex: '999',
      overflow: 'hidden',
    },
    chatHeader: {
      background: 'linear-gradient(135deg, #3b82f6 0%, #8b5cf6 100%)',
      padding: '20px',
      color: 'white',
      display: 'flex',
      justifyContent: 'space-between',
      alignItems: 'center',
    },
    chatHeaderTitle: {
      fontSize: '1.2rem',
      fontWeight: '700',
      margin: '0',
    },
    chatBody: {
      flex: '1',
      overflowY: 'auto',
      padding: '20px',
      display: 'flex',
      flexDirection: 'column',
      gap: '12px',
    },
    chatInputArea: {
      padding: '16px',
      borderTop: '1px solid rgba(148, 163, 184, 0.2)',
      display: 'flex',
      gap: '10px',
    },
    chatInput: {
      flex: '1',
      padding: '12px 16px',
      borderRadius: '10px',
      border: '1px solid rgba(148, 163, 184, 0.3)',
      background: '#0f172a',
      color: 'white',
      fontSize: '1rem',
    },
    chatSendBtn: {
      padding: '12px 20px',
      borderRadius: '10px',
      background: 'linear-gradient(135deg, #3b82f6 0%, #8b5cf6 100%)',
      border: 'none',
      color: 'white',
      fontWeight: '600',
      cursor: 'pointer',
      transition: 'all 0.3s ease',
    },
    userMessage: {
      alignSelf: 'flex-end',
      background: 'linear-gradient(135deg, #3b82f6 0%, #8b5cf6 100%)',
      color: 'white',
      padding: '12px 18px',
      borderRadius: '18px',
      maxWidth: '75%',
      wordWrap: 'break-word',
    },
    botMessage: {
      alignSelf: 'flex-start',
      background: '#0f172a',
      color: '#e2e8f0',
      padding: '12px 18px',
      borderRadius: '18px',
      maxWidth: '75%',
      wordWrap: 'break-word',
    },
  };

  return (
    <main>
      <style>{`
        * { box-sizing: border-box; }
        
        @keyframes fadeInUp {
          from {
            opacity: 0;
            transform: translateY(60px);
          }
          to {
            opacity: 1;
            transform: translateY(0);
          }
        }
        
        @keyframes fadeInScale {
          from {
            opacity: 0;
            transform: scale(0.9);
          }
          to {
            opacity: 1;
            transform: scale(1);
          }
        }
        
        @keyframes slideInLeft {
          from {
            opacity: 0;
            transform: translateX(-60px);
          }
          to {
            opacity: 1;
            transform: translateX(0);
          }
        }
        
        @keyframes slideInRight {
          from {
            opacity: 0;
            transform: translateX(60px);
          }
          to {
            opacity: 1;
            transform: translateX(0);
          }
        }
        
        @keyframes float {
          0%, 100% { transform: translateY(0px); }
          50% { transform: translateY(-20px); }
        }
        
        @keyframes pulse {
          0%, 100% { 
            box-shadow: 0 8px 30px rgba(59, 130, 246, 0.6);
            transform: scale(1);
          }
          50% { 
            box-shadow: 0 12px 50px rgba(59, 130, 246, 0.9);
            transform: scale(1.05);
          }
        }
        
        @keyframes shimmer {
          0% { background-position: -1000px 0; }
          100% { background-position: 1000px 0; }
        }
        
        @keyframes gradient {
          0%, 100% { background-position: 0% 50%; }
          50% { background-position: 100% 50%; }
        }
        
        .scroll-reveal {
          opacity: 0;
          transform: translateY(40px);
        }
        
        .reveal-animation {
          animation: fadeInUp 1s ease forwards;
        }
        
        .feature-card {
          transition: all 0.5s cubic-bezier(0.34, 1.56, 0.64, 1);
        }
        
        .feature-card:hover {
          transform: translateY(-16px) scale(1.03);
          box-shadow: 0 25px 60px rgba(59, 130, 246, 0.5);
          border-color: rgba(59, 130, 246, 0.8);
        }
        
        .feature-card:hover .feature-icon {
          animation: float 2s ease-in-out infinite;
        }
        
        .module {
          transition: all 0.4s cubic-bezier(0.34, 1.56, 0.64, 1);
        }
        
        .module:hover {
          transform: translateY(-12px) scale(1.04) rotateZ(1deg);
          box-shadow: 0 30px 70px rgba(139, 92, 246, 0.6);
        }
        
        .button-primary {
          transition: all 0.4s cubic-bezier(0.34, 1.56, 0.64, 1);
          position: relative;
          overflow: hidden;
        }
        
        .button-primary::before {
          content: '';
          position: absolute;
          top: 0;
          left: -100%;
          width: 100%;
          height: 100%;
          background: linear-gradient(90deg, transparent, rgba(255,255,255,0.3), transparent);
          transition: left 0.5s;
        }
        
        .button-primary:hover::before {
          left: 100%;
        }
        
        .button-primary:hover {
          transform: translateY(-5px) scale(1.05);
          box-shadow: 0 15px 40px rgba(255,255,255,0.4);
        }
        
        .button-primary:active {
          transform: translateY(-2px) scale(1.02);
        }
        
        .button-secondary {
          transition: all 0.4s cubic-bezier(0.34, 1.56, 0.64, 1);
        }
        
        .button-secondary:hover {
          background: rgba(255, 255, 255, 0.2);
          transform: translateY(-5px) scale(1.05);
          box-shadow: 0 12px 35px rgba(255, 255, 255, 0.2);
        }
        
        .button-secondary:active {
          transform: translateY(-2px) scale(1.02);
        }
        
        .chat-button {
          animation: pulse 3s ease-in-out infinite;
          transition: all 0.3s ease;
        }
        
        .chat-button:hover {
          transform: scale(1.15) rotate(5deg);
          animation: none;
          box-shadow: 0 15px 50px rgba(59, 130, 246, 0.9);
        }
        
        .chat-button:active {
          transform: scale(1.05);
        }
        
        .chat-send-btn {
          transition: all 0.3s cubic-bezier(0.34, 1.56, 0.64, 1);
        }
        
        .chat-send-btn:hover {
          transform: scale(1.1) rotate(-3deg);
          box-shadow: 0 10px 25px rgba(59, 130, 246, 0.7);
        }
        
        .chat-send-btn:active {
          transform: scale(1.05);
        }
        
        .chat-widget {
          animation: slideInRight 0.5s cubic-bezier(0.34, 1.56, 0.64, 1);
        }
        
        .user-message {
          animation: slideInRight 0.4s ease;
        }
        
        .bot-message {
          animation: slideInLeft 0.4s ease;
        }
        
        .modal {
          animation: fadeInScale 0.4s cubic-bezier(0.34, 1.56, 0.64, 1);
        }
        
        .hero-content h1 {
          animation: fadeInUp 0.8s ease 0.2s both;
        }
        
        .hero-content p {
          animation: fadeInUp 0.8s ease 0.4s both;
        }
        
        .hero-buttons {
          animation: fadeInUp 0.8s ease 0.6s both;
        }
        
        .feature-card:nth-child(1) { animation-delay: 0.1s; }
        .feature-card:nth-child(2) { animation-delay: 0.2s; }
        .feature-card:nth-child(3) { animation-delay: 0.3s; }
        .feature-card:nth-child(4) { animation-delay: 0.4s; }
        .feature-card:nth-child(5) { animation-delay: 0.5s; }
        .feature-card:nth-child(6) { animation-delay: 0.6s; }
        
        .module:nth-child(1) { animation-delay: 0.1s; }
        .module:nth-child(2) { animation-delay: 0.2s; }
        .module:nth-child(3) { animation-delay: 0.3s; }
        .module:nth-child(4) { animation-delay: 0.4s; }
        
        @media (max-width: 768px) {
          .chat-widget { width: 90%; max-width: 400px; }
          .feature-grid { grid-template-columns: 1fr; }
          .module-grid { grid-template-columns: 1fr; }
        }
        
        @media (prefers-reduced-motion: reduce) {
          *, *::before, *::after {
            animation-duration: 0.01ms !important;
            animation-iteration-count: 1 !important;
            transition-duration: 0.01ms !important;
          }
        }
      `}</style>

      {/* Hero Section */}
      <section style={styles.hero}>
        <div style={styles.heroContent} className="hero-content">
          <h1 style={styles.title}>Physical AI & Humanoid Robotics</h1>
          <p style={styles.subtitle}>
            Master the art of bridging digital intelligence with physical embodiment. Build autonomous systems that think, see, and act.
          </p>
          <div style={styles.heroButtons} className="hero-buttons">
            <a 
              href="/docs/01-intro-to-ros2" 
              style={{...styles.button, ...styles.buttonPrimary}}
              className="button-primary"
            >
              Start Learning →
            </a>
            <a 
              href="https://github.com/MUKARRAM-ONE/AI-Robotic-Book/tree/main/AI_Robotics_book/docs" 
              style={{...styles.button, ...styles.buttonSecondary}}
              className="button-secondary"
            >
              View on GitHub
            </a>
            <a
              href="/auth?action=login"
              style={{...styles.button, ...styles.buttonSecondary}}
              className="button-secondary"
            >
              Sign In
            </a>
            <a
              href="/auth?action=signup"
              style={{...styles.button, ...styles.buttonPrimary}}
              className="button-primary"
            >
              Create Account
            </a>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section style={styles.features} className="scroll-reveal">
        <h2 style={styles.featuresTitle}>What You'll Master</h2>
        <p style={styles.featuresSubtitle}>
          Comprehensive curriculum designed to transform you into a robotics expert
        </p>
        <div style={styles.featureGrid}>
          {[
            { icon: '🤖', title: 'ROS 2 Fundamentals', desc: 'Master the Robot Operating System with hands-on examples and real-world applications.' },
            { icon: '🎮', title: 'Digital Twins', desc: 'Build virtual representations of robots using Gazebo and Isaac Sim.' },
            { icon: '🧠', title: 'AI Integration', desc: 'Implement intelligent behaviors using machine learning and planning algorithms.' },
            { icon: '👁️', title: 'Vision & Language', desc: 'Integrate VLMs and LLMs to enable robots to see, understand, and reason.' },
            { icon: '📚', title: '13 Chapters', desc: 'Comprehensive curriculum from basics to advanced robotics concepts.' },
            { icon: '💻', title: 'Hands-On Code', desc: 'Learn by doing with complete code examples and interactive tutorials.' }
          ].map((feature, idx) => (
            <div key={idx} style={styles.featureCard} className="feature-card scroll-reveal">
              <div style={styles.featureIcon} className="feature-icon">{feature.icon}</div>
              <h3 style={styles.featureTitle}>{feature.title}</h3>
              <p style={styles.featureDesc}>{feature.desc}</p>
            </div>
          ))}
        </div>
      </section>

      {/* Modules Section */}
      <section style={styles.modules} className="scroll-reveal">
        <h2 style={styles.featuresTitle}>Course Structure</h2>
        <p style={styles.featuresSubtitle}>Four comprehensive modules from fundamentals to advanced applications</p>
        <div style={styles.moduleGrid}>
          {[
            { module: 1, title: 'ROS 2 Fundamentals', chapters: ['Introduction to ROS 2', 'Nodes, Topics & Services', 'Python Integration'] },
            { module: 2, title: 'Digital Twin & Simulation', chapters: ['URDF for Humanoids', 'Gazebo Fundamentals', 'Unity Integration', 'Sensor Simulation', 'Isaac Sim Intro'] },
            { module: 3, title: 'AI-Robot Brain', chapters: ['Isaac ROS vSLAM', 'Nav2 Path Planning'] },
            { module: 4, title: 'Vision Language Models', chapters: ['VLM with Whisper', 'LLM Planning', 'Capstone Project'] }
          ].map((mod, idx) => (
            <div key={idx} style={styles.module} className="module scroll-reveal">
              <h3 style={styles.moduleNumber}>MODULE {mod.module}</h3>
              <h4 style={styles.moduleTitle}>{mod.title}</h4>
              <ul style={styles.moduleList}>
                {mod.chapters.map((ch, i) => (
                  <li key={i} style={styles.moduleItem}>→ {ch}</li>
                ))}
              </ul>
            </div>
          ))}
        </div>
      </section>

      {/* CTA Section */}
      <section style={styles.cta} className="scroll-reveal">
        <h2 style={styles.ctaTitle}>Ready to Build Intelligent Robots?</h2>
        <p style={styles.ctaText}>Start with the fundamentals and progress to building your own autonomous systems.</p>
        <a 
          href="/docs/01-intro-to-ros2" 
          style={{...styles.button, ...styles.buttonPrimary, fontSize: '1.2rem', padding: '18px 48px'}}
          className="button-primary"
        >
          Get Started Now
        </a>
      </section>

      {/* Contact Section */}
      <section style={styles.contact} className="scroll-reveal">
        <h2 style={styles.contactTitle}>Get in Touch</h2>
        <p style={styles.contactText}>Have questions? Reach out via GitHub, LinkedIn, or send a quick message.</p>
        <div style={styles.contactButtons}>
          <a 
            href="https://github.com/MUKARRAM-ONE/AI-Robotic-Book/tree/main/AI_Robotics_book/docs" 
            target="_blank" 
            rel="noreferrer" 
            style={{...styles.button, ...styles.buttonSecondary}}
            className="button-secondary"
          >
            GitHub
          </a>
          <a 
            href="https://www.linkedin.com/in/your-linkedin" 
            target="_blank" 
            rel="noreferrer" 
            style={{...styles.button, ...styles.buttonSecondary}}
            className="button-secondary"
          >
            LinkedIn
          </a>
          <button 
            style={{...styles.button, ...styles.buttonPrimary}}
            onClick={() => setIsOpen(true)}
            className="button-primary"
          >
            Send Message
          </button>
        </div>
      </section>

      {/* Contact Modal */}
      {isOpen && (
        <div style={styles.modalOverlay} onClick={() => setIsOpen(false)}>
          <div style={styles.modal} className="modal" onClick={(e) => e.stopPropagation()}>
            <h3 style={styles.modalTitle}>Send a Message</h3>
            <label style={styles.label}>Name</label>
            <input style={styles.input} value={name} onChange={(e) => setName(e.target.value)} placeholder="Your name" />
            <label style={styles.label}>Email</label>
            <input style={styles.input} value={email} onChange={(e) => setEmail(e.target.value)} placeholder="your@email.com" />
            <label style={styles.label}>Message</label>
            <textarea style={styles.textarea} value={message} onChange={(e) => setMessage(e.target.value)} placeholder="How can we help you?" />
            <div style={{display: 'flex', gap: 12, marginTop: 24}}>
              <button 
                style={{...styles.button, ...styles.buttonPrimary, flex: 1}}
                className="button-primary"
                onClick={() => {
                  const subject = encodeURIComponent('Contact from AI Robotics site');
                  const body = encodeURIComponent(`Name: ${name}\nEmail: ${email}\n\n${message}`);
                  window.location.href = `mailto:youremail@example.com?subject=${subject}&body=${body}`;
                }}
              >
                Send Email
              </button>
              <button 
                style={{...styles.button, ...styles.buttonSecondary}}
                onClick={() => setIsOpen(false)}
                className="button-secondary"
              >
                Close
              </button>
            </div>
          </div>
        </div>
      )}
    </main>
  );
}