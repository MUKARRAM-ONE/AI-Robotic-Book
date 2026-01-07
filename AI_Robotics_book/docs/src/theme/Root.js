import React from 'react';
import { AuthProvider } from '../context/AuthContext';
import PaywallGate from '../components/PaywallGate/PaywallGate';
import ChatAssistant from '@site/src/components/ChatAssistant/ChatAssistant';

// Wrap the entire site: auth context -> paywall gating -> chat assistant
export default function Root({ children }) {
  return (
    <AuthProvider>
      <PaywallGate>
        <>
          {children}
          <ChatAssistant />
        </>
      </PaywallGate>
    </AuthProvider>
  );
}
