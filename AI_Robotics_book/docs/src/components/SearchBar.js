import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './SearchBar.module.css';

export default function SearchBar({ navbar = false }) {
  const [query, setQuery] = useState('');
  const [results, setResults] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [showResults, setShowResults] = useState(false);
  const { siteConfig } = useDocusaurusContext();
  const searchRef = useRef(null);
  const inputRef = useRef(null);

  // Handle keyboard shortcuts (Ctrl+K)
  useEffect(() => {
    const handleKeyDown = (e) => {
      if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
        e.preventDefault();
        if (inputRef.current) {
          inputRef.current.focus();
          setShowResults(true);
        }
      }
      if (e.key === 'Escape') {
        setShowResults(false);
        if (inputRef.current) {
          inputRef.current.blur();
        }
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, []);

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (searchRef.current && !searchRef.current.contains(event.target)) {
        setShowResults(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  const handleSearch = async (e) => {
    e.preventDefault();
    if (!query.trim()) return;

    setLoading(true);
    setError(null);
    setResults(null);
    setShowResults(true);

    try {
      const ragApiUrl = siteConfig.customFields.ragApiUrl || 'http://localhost:8001';
      const response = await fetch(`${ragApiUrl}/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include',
        body: JSON.stringify({ query }),
      });

      if (!response.ok) {
        if (response.status === 401) {
          throw new Error('Please sign in to use the search feature');
        }
        throw new Error(`API Error: ${response.status}`);
      }

      const data = await response.json();
      setResults(data);
    } catch (err) {
      setError(err.message || 'Failed to fetch search results. Make sure RAG backend is running.');
      console.error('Search error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div 
      ref={searchRef}
      className={`${styles.searchContainer} ${navbar ? styles.navbarSearch : ''}`}
    >
      <form onSubmit={handleSearch} className={styles.searchForm}>
        <input
          ref={inputRef}
          type="text"
          placeholder={navbar ? "Search" : "Ask anything about AI Robotics..."}
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          onFocus={() => setShowResults(true)}
          className={styles.searchInput}
        />
        {navbar && (
          <div className={styles.keyboardShortcut}>
            <span className={styles.keyboardKey}>Ctrl</span>
            <span>K</span>
          </div>
        )}
        <button type="submit" className={styles.searchButton} disabled={loading}>
          {loading ? '⏳' : '🔍'}
        </button>
      </form>

      {navbar ? (
        showResults && (results || error) && (
          <div className={styles.dropdownResults}>
            {error && <div className={styles.error}>{error}</div>}
            {results && (
              <div className={styles.answer}>
                <p>{results.answer || 'No answer generated'}</p>
                {results.sources && results.sources.length > 0 && (
                  <div style={{ marginTop: '1rem', fontSize: '0.85rem', color: '#737373' }}>
                    <strong>Sources:</strong> {results.sources.length} document(s)
                  </div>
                )}
              </div>
            )}
          </div>
        )
      ) : (
        <>
          {error && <div className={styles.error}>{error}</div>}
          {results && (
            <div className={styles.results}>
              <div className={styles.answer}>
                <h4>Answer:</h4>
                <p>{results.answer || 'No answer generated'}</p>
              </div>
              {results.sources && results.sources.length > 0 && (
                <div className={styles.sources}>
                  <h5>Sources:</h5>
                  <ul>
                    {results.sources.map((source, idx) => (
                      <li key={idx}>
                        <strong>{source.source || 'Unknown'}</strong>
                        <p>{source.text || source.content}</p>
                      </li>
                    ))}
                  </ul>
                </div>
              )}
            </div>
          )}
        </>
      )}
    </div>
  );
}
