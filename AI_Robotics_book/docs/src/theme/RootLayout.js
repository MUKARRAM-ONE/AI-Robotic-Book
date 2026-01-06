import React from 'react';
import { useLocation } from '@docusaurus/router';
import ChatAssistant from '@site/src/components/ChatAssistant/ChatAssistant';

export default function RootLayout({ children }) {
  return (
    <>
      {children}
      <ChatAssistant />
    </>
  );
}
