import React from 'react';
import { AuthProvider } from '../context/AuthContext';
import PaywallGate from '../components/PaywallGate/PaywallGate';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <AuthProvider>
      <PaywallGate>
        <>
          {children}
        </>
      </PaywallGate>
    </AuthProvider>
  );
}
