/**
 * Root component - wraps entire Docusaurus app
 * Used to inject global components like ChatBot
 */

import React from 'react';
import ChatBot from '../components/ChatBot/ChatBot';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatBot />
    </>
  );
}
