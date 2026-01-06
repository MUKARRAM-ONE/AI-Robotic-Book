import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatAssistant.module.css';

export default function ChatAssistant() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const messagesEndRef = useRef(null);

  useEffect(() => {
    // Check if user is logged in
    const token = localStorage.getItem('access_token');
    setIsLoggedIn(!!token);
  }, []);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSendMessage = async (e) => {
    e.preventDefault();
    
    if (!input.trim()) return;

    if (!isLoggedIn) {
      setMessages([...messages, {
        id: Date.now(),
        text: 'Please log in to use AI Book Assistance. Click the Sign In button in the top right.',
        sender: 'bot'
      }]);
      setInput('');
      return;
    }

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: input,
      sender: 'user'
    };
    
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      // Call backend RAG API
      const token = localStorage.getItem('access_token');
      const response = await fetch('https://your-azure-vm-ip:8000/search', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify({ query: input })
      });

      if (response.ok) {
        const data = await response.json();
        const botMessage = {
          id: Date.now() + 1,
          text: data.answer || 'No results found. Please try another query.',
          sender: 'bot',
          references: data.sources || []
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        setMessages(prev => [...prev, {
          id: Date.now() + 1,
          text: 'Error connecting to AI assistant. Please try again.',
          sender: 'bot'
        }]);
      }
    } catch (error) {
      setMessages(prev => [...prev, {
        id: Date.now() + 1,
        text: 'Error: Could not reach the AI assistant. Make sure backend is running.',
        sender: 'bot'
      }]);
    }

    setLoading(false);
  };

  return (
    <div className={styles.chatContainer}>
      {/* Chat Button */}
      <button
        className={styles.chatButton}
        onClick={() => setIsOpen(!isOpen)}
        title="AI Book Assistant"
      >
        💬
      </button>

      {/* Chat Widget */}
      {isOpen && (
        <div className={styles.chatWidget}>
          <div className={styles.chatHeader}>
            <h3>📚 AI Book Assistant</h3>
            <button
              className={styles.closeBtn}
              onClick={() => setIsOpen(false)}
            >
              ✕
            </button>
          </div>

          <div className={styles.messagesContainer}>
            {messages.length === 0 ? (
              <div className={styles.emptyState}>
                <p>👋 Hello! I'm your AI Book Assistant.</p>
                <p>{isLoggedIn 
                  ? 'Ask me anything about the robotics textbook!' 
                  : 'Please log in to start chatting.'}</p>
              </div>
            ) : (
              messages.map(msg => (
                <div key={msg.id} className={`${styles.message} ${styles[msg.sender]}`}>
                  <div className={styles.messageText}>{msg.text}</div>
                  {msg.references && msg.references.length > 0 && (
                    <div className={styles.references}>
                      <strong>📄 References:</strong>
                      <ul>
                        {msg.references.map((ref, idx) => (
                          <li key={idx}>
                            <a href={`/AI-Robotic-Book/docs/${ref.page}`}>
                              {ref.page}
                            </a>
                          </li>
                        ))}
                      </ul>
                    </div>
                  )}
                </div>
              ))
            )}
            {loading && (
              <div className={`${styles.message} ${styles.bot}`}>
                <div className={styles.typing}>
                  <span></span><span></span><span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSendMessage} className={styles.inputForm}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder={isLoggedIn ? "Ask about the textbook..." : "Log in to chat"}
              disabled={!isLoggedIn || loading}
              className={styles.input}
            />
            <button
              type="submit"
              disabled={!isLoggedIn || loading || !input.trim()}
              className={styles.sendBtn}
            >
              Send
            </button>
          </form>
        </div>
      )}
    </div>
  );
}
