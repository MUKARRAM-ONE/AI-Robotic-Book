import React, { useState, useEffect } from 'react';

const AuthButton = () => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchUser = async () => {
      try {
        const token = localStorage.getItem('access_token');
        if (!token) {
          setLoading(false);
          return;
        }
        
        const response = await fetch('https://my-rag-server.centralindia.cloudapp.azure.com/users/me', {
          headers: {
            'Authorization': `Bearer ${token}`,
          },
        });
        if (response.ok) {
          const data = await response.json();
          setUser(data);
        } else {
          localStorage.removeItem('access_token');
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
        const token = localStorage.getItem('access_token');
        const response = await fetch('https://my-rag-server.centralindia.cloudapp.azure.com/auth/jwt/logout', {
        method: 'POST',
        headers: token ? {
          'Authorization': `Bearer ${token}`,
        } : {},
      });

      if (response.ok) {
        localStorage.removeItem('access_token');
        setUser(null);
        window.location.reload();
      }
    } catch (error) {
      console.error('Logout failed', error);
      // Clear token anyway
      localStorage.removeItem('access_token');
      setUser(null);
      window.location.reload();
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
            href="/AI-Robotic-Book/auth?action=login"
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
            href="/AI-Robotic-Book/auth?action=signup"
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
