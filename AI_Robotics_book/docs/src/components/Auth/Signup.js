import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import styles from './Auth.module.css';

const Signup = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');
  const [retries, setRetries] = useState(0);
  const history = useHistory();
  const API_URL = 'https://my-rag-server.centralindia.cloudapp.azure.com';

  const handleSignup = async (e) => {
    e.preventDefault();
    setError('');
    setSuccess('');

    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (password.length < 8) {
      setError('Password must be at least 8 characters');
      return;
    }

    setLoading(true);

    const attempt = async () => {
      try {
        const response = await fetch(`${API_URL}/auth/register`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          credentials: 'include',
          body: JSON.stringify({ email, password }),
        });

        if (response.ok) {
          setSuccess('Account created! Redirecting to login...');
          setTimeout(() => {
            history.push('/AI-driven-development/auth?action=login');
          }, 1500);
        } else {
          let detail = 'Signup failed. Please check your details.';
          try {
            const data = await response.json();
            if (data && data.detail) detail = Array.isArray(data.detail) ? data.detail[0].msg || JSON.stringify(data.detail) : data.detail;
          } catch {}
          setError(detail);
        }
      } catch (err) {
        if (retries < 2) {
          setRetries((r) => r + 1);
          setError('Connecting to the server… retrying');
          setTimeout(attempt, 800);
          return;
        }
        setError('Cannot reach the server. Please start the backend on port 8001 and try again.');
      } finally {
        setLoading(false);
      }
    };

    attempt();
  };

  return (
    <div className={styles.authContainer}>
      <div className={styles.authBox}>
        <h2>Create Account</h2>
        {error && <div className={styles.alert + ' ' + styles.alertError}>{error}</div>}
        {success && <div className={styles.alert + ' ' + styles.alertSuccess}>{success}</div>}
        <form onSubmit={handleSignup}>
          <div className={styles.formGroup}>
            <input
              type="email"
              placeholder="Email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
              autoComplete="email"
            />
          </div>
          <div className={styles.formGroup}>
            <input
              type="password"
              placeholder="Password (min 8 characters)"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
              minLength="8"
              autoComplete="new-password"
            />
          </div>
          <div className={styles.formGroup}>
            <input
              type="password"
              placeholder="Confirm Password"
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              required
              autoComplete="new-password"
            />
          </div>
          <button type="submit" disabled={loading} className={styles.submitBtn}>
            {loading ? 'Creating account...' : 'Create Account'}
          </button>
        </form>
        
        <div className={styles.divider}>OR</div>
        
        <div className={styles.oauthButtons}>
          <a href={`${API_URL}/auth/google/authorize`} className={styles.oauthBtn}>
            🔵 Google
          </a>
          <a href={`${API_URL}/auth/github/authorize`} className={styles.oauthBtn}>
            ⚫ GitHub
          </a>
        </div>
        
        <p className={styles.toggleAuth}>
          Already have an account? <a href="/auth?action=login">Sign in</a>
        </p>
      </div>
    </div>
  );
};

export default Signup;
