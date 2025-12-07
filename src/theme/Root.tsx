/**
 * Root component - wraps entire Docusaurus app
 * Used to inject global components like ChatBot and AuthPanel
 */

import React from 'react';
import ChatBot from '../components/ChatBot/ChatBot';
import AuthPanel from '../components/AuthPanel/AuthPanel';

export default function Root({children}) {
  return (
    <>
      {children}
      <AuthPanel />
      <ChatBot />
    </>
  );
}
