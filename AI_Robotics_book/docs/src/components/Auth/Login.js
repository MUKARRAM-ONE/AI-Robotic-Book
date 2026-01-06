import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import styles from './Auth.module.css';

const Login = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [tried, setTried] = useState(false);
  const history = useHistory();
  const API_URL = 'https://my-rag-server.centralindia.cloudapp.azure.com';

  const attemptLogin = async () => {
    setLoading(true);
    setError('');

    try {
      const response = await fetch(`${API_URL}/auth/jwt/login`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
          'Accept': 'application/json',
        },
        // Using Bearer tokens; cookies not required
        body: new URLSearchParams({
          username: email,
          password: password,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        if (data.access_token) {
          localStorage.setItem('access_token', data.access_token);
        }
        window.location.href = '/docs/01-intro-to-ros2';
      } else {
        let detail = 'Login failed. Please check your credentials.';
        try {
          const data = await response.json();
          if (data && data.detail) detail = Array.isArray(data.detail) ? data.detail[0].msg || JSON.stringify(data.detail) : data.detail;
        } catch {}
        setError(detail);
      }
    } catch (err) {
      if (!tried) {
        setTried(true);
        setError('Connecting to the server… retrying');
        setTimeout(() => attemptLogin(), 800);
        return;
      }
      setError('Cannot reach the server. Please ensure the backend is reachable at the configured HTTPS URL.');
    } finally {
      setLoading(false);
    }
  };

  const handleLogin = async (e) => {
    e.preventDefault();
    await attemptLogin();
  };

  return (
    <div className={styles.authContainer}>
      <div className={styles.authBox}>
        <h2>Sign In</h2>
        {error && <div className={styles.alert + ' ' + styles.alertError}>{error}</div>}
        <form onSubmit={handleLogin}>
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
              placeholder="Password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
              autoComplete="current-password"
            />
          </div>
          <button type="submit" disabled={loading} className={styles.submitBtn}>
            {loading ? 'Signing in...' : 'Sign In'}
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
          Don't have an account? <a href="/auth?action=signup">Sign up</a>
        </p>
      </div>
    </div>
  );
};

export default Login;
