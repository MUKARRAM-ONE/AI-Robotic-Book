import React, { useState, useEffect } from 'react';

const AuthButton = () => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchUser = async () => {
      try {
        const response = await fetch('http://20.197.8.180/users/me', {
          credentials: 'include',
        });
        console.log('Auth check response:', response.status);
        if (response.ok) {
          const data = await response.json();
          console.log('User data:', data);
          setUser(data);
        } else {
          console.log('Not authenticated:', response.status);
        }
      } catch (error) {
        console.error('Failed to fetch user', error);
      } finally {
        setLoading(false);
      }
    };
    fetchUser();
  }, []);

  const handleLogout = async () => {
    try {
        const response = await fetch('http://20.197.8.180/auth/jwt/logout', {
        method: 'POST',
        credentials: 'include',
      });

      if (response.ok) {
        setUser(null);
        window.location.reload();
      }
    } catch (error) {
      console.error('Logout failed', error);
    }
  };

  if (loading) {
    return null;
  }

  return (
    <div style={{
      display: 'flex',
      gap: '10px',
      alignItems: 'center',
      marginLeft: '20px',
    }}>
      {user ? (
        <>
          <span style={{ color: '#ff6b35', fontSize: '14px', fontWeight: '600' }}>
            {user.email}
          </span>
          <button
            onClick={handleLogout}
            style={{
              padding: '6px 12px',
              background: '#ff6b35',
              color: 'white',
              border: 'none',
              borderRadius: '6px',
              cursor: 'pointer',
              fontSize: '14px',
              fontWeight: '600',
              transition: 'all 0.3s ease',
            }}
            onMouseOver={(e) => {
              e.target.style.background = '#ff8555';
              e.target.style.transform = 'translateY(-2px)';
            }}
            onMouseOut={(e) => {
              e.target.style.background = '#ff6b35';
              e.target.style.transform = 'translateY(0)';
            }}
          >
            Logout
          </button>
        </>
      ) : (
        <>
          <a
            href="/AI-driven-development/auth?action=login"
            style={{
              padding: '6px 12px',
              border: '2px solid #ff6b35',
              color: '#ff6b35',
              borderRadius: '6px',
              textDecoration: 'none',
              cursor: 'pointer',
              fontSize: '14px',
              fontWeight: '600',
              transition: 'all 0.3s ease',
            }}
            onMouseOver={(e) => {
              e.target.style.background = 'rgba(255, 107, 53, 0.1)';
              e.target.style.transform = 'translateY(-2px)';
            }}
            onMouseOut={(e) => {
              e.target.style.background = 'transparent';
              e.target.style.transform = 'translateY(0)';
            }}
          >
            Sign In
          </a>
          <a
            href="/AI-driven-development/auth?action=signup"
            style={{
              padding: '6px 12px',
              background: '#ff6b35',
              color: 'white',
              border: 'none',
              borderRadius: '6px',
              textDecoration: 'none',
              cursor: 'pointer',
              fontSize: '14px',
              fontWeight: '600',
              transition: 'all 0.3s ease',
            }}
            onMouseOver={(e) => {
              e.target.style.background = '#ff8555';
              e.target.style.transform = 'translateY(-2px)';
            }}
            onMouseOut={(e) => {
              e.target.style.background = '#ff6b35';
              e.target.style.transform = 'translateY(0)';
            }}
          >
            Sign Up
          </a>
        </>
      )}
    </div>
  );
};

export default AuthButton;
