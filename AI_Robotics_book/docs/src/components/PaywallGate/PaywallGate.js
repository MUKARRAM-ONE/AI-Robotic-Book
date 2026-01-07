import React, { useState, useEffect } from 'react';
import { useAuth } from '../../context/AuthContext';
import styles from './PaywallGate.module.css';

const PaywallGate = ({ children }) => {
  const { user, loading } = useAuth();
  const [scrollPercentage, setScrollPercentage] = useState(0);
  const [showPaywall, setShowPaywall] = useState(false);
  const [guestContinue, setGuestContinue] = useState(false);

  useEffect(() => {
    const path = typeof window !== 'undefined' ? window.location.pathname : '';
    const isHome = path === '/' || path === '/AI-Robotic-Book' || path === '/AI-Robotic-Book/';
    const isAuth = path.startsWith('/AI-Robotic-Book/auth') || path.startsWith('/auth');

    // Do not enforce paywall on homepage or auth pages
    if (isHome || isAuth) {
      setShowPaywall(false);
      return;
    }

    const handleScroll = () => {
      const scrollTop = window.scrollY;
      const docHeight = document.documentElement.scrollHeight - window.innerHeight;
      const scrolled = docHeight > 0 ? (scrollTop / docHeight) * 100 : 0;
      setScrollPercentage(scrolled);

      // Lock paywall after 40% scroll for unauthenticated users (dismissable for guests)
      if (scrolled > 40 && !user && !guestContinue) {
        setShowPaywall(true);
      }
    };

    window.addEventListener('scroll', handleScroll);

    // Run once after load to catch cases where user already scrolled past threshold
    handleScroll();

    return () => window.removeEventListener('scroll', handleScroll);
  }, [user, guestContinue]);

  // Disable background scroll when paywall is active and user not authenticated
  useEffect(() => {
    if (showPaywall && !user) {
      const originalOverflow = document.body.style.overflow;
      document.body.style.overflow = 'hidden';
      return () => {
        document.body.style.overflow = originalOverflow;
      };
    }
  }, [showPaywall, user]);

  const path = typeof window !== 'undefined' ? window.location.pathname : '';
  const isHome = path === '/' || path === '/AI-Robotic-Book' || path === '/AI-Robotic-Book/';
  const isAuth = path.startsWith('/AI-Robotic-Book/auth') || path.startsWith('/auth');

  // If loading or public pages, just render content
  if (loading || isHome || isAuth || user) return <div>{children}</div>;

  const handleLoginClick = () => {
    window.location.href = '/AI-Robotic-Book/auth?action=login';
  };

  const handleSignupClick = () => {
    window.location.href = '/AI-Robotic-Book/auth?action=signup';
  };

  const handleContinueGuest = () => {
    setGuestContinue(true);
    setShowPaywall(false);
  };

  return (
    <div className={styles.contentWrapper}>
      <div className={`${styles.contentArea} ${showPaywall && !user ? styles.blurred : ''}`}>
        {children}
      </div>

      {showPaywall && !user && (
        <div className={styles.paywallOverlay}>
          <div className={styles.paywallModal}>
            <div className={styles.paywallContent}>
              <h2>You’re reading as a guest</h2>
              <p>You can keep reading without signing up. Log in or sign up to unlock pro features and upcoming drops (marketing, new content, more).</p>
              
              <div className={styles.authButtons}>
                <button 
                  className={styles.loginBtn}
                  onClick={handleLoginClick}
                >
                  Sign In
                </button>
                <button 
                  className={styles.signupBtn}
                  onClick={handleSignupClick}
                >
                  Create Account
                </button>
                <button
                  className={styles.guestBtn}
                  onClick={handleContinueGuest}
                >
                  Continue as guest
                </button>
              </div>

              <div className={styles.benefits}>
                <h4>Benefits of signing in:</h4>
                <ul>
                  <li>📚 Full access to all chapters</li>
                  <li>🔍 Access to RAG-powered search</li>
                  <li>💬 Community discussions</li>
                  <li>📊 Progress tracking</li>
                </ul>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default PaywallGate;
