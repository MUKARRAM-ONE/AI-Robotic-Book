import React, {useState, useRef} from 'react';
import styles from './chat.module.css';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function ChatWidget(){
  const {siteConfig} = useDocusaurusContext();
  const ragUrl = siteConfig.customFields?.ragApiUrl || '';
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const listRef = useRef(null);

  const postQuery = async (q) => {
    if(!ragUrl){
      setMessages(m => [...m, {role:'system',text:'RAG backend not configured. Set siteConfig.customFields.ragApiUrl.'}]);
      return;
    }
    setLoading(true);
    setMessages(m => [...m, {role:'user', text: q}]);
    try{
      const res = await fetch(new URL('/search', ragUrl).toString(), {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({query: q})
      });
      const data = await res.json();
      if(res.ok){
        setMessages(m => [...m, {role:'assistant', text: data.answer || 'No answer.'}]);
      } else {
        setMessages(m => [...m, {role:'assistant', text: `Error: ${data.detail || res.statusText}`}]);
      }
    }catch(e){
      setMessages(m => [...m, {role:'assistant', text: `Network error: ${e.message}`}]);
    }finally{
      setLoading(false);
      setTimeout(()=>{ if(listRef.current) listRef.current.scrollTop = listRef.current.scrollHeight; }, 50);
    }
  };

  const onSend = () => {
    if(!input.trim()) return;
    const q = input.trim();
    setInput('');
    postQuery(q);
  };

  return (
    <div className={styles.widget}>
      <div className={styles.header}>AI Chat — Ask about this book</div>
      <div className={styles.messages} ref={listRef}>
        {messages.map((m,i)=> (
          <div key={i} className={m.role==='user' ? styles.msgUser : (m.role==='assistant'? styles.msgAssistant: styles.msgSystem)}>
            {m.text}
          </div>
        ))}
      </div>
      <div className={styles.controls}>
        <input className={styles.input} value={input} onChange={e=>setInput(e.target.value)} placeholder={ragUrl? 'Ask a question about the book...' : 'RAG backend not configured'} onKeyDown={e=>{ if(e.key==='Enter') onSend(); }} />
        <button className={styles.send} onClick={onSend} disabled={loading || !ragUrl}>{loading? '...' : 'Send'}</button>
      </div>
    </div>
  );
}
