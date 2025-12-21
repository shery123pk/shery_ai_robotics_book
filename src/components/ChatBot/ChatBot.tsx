/**
 * ChatBot component - RAG-powered AI assistant for textbook
 */

import React, { useState, useEffect, useRef } from 'react';
import { getApiUrl } from '../../utils/apiConfig';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  timestamp: Date;
}

interface Citation {
  module: string;
  chapter: string;
  section?: string;
}

export default function ChatBot(): React.JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId] = useState(() => `session-${Date.now()}-${Math.random()}`);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Text selection state
  const [selectedText, setSelectedText] = useState('');
  const [showSelectionButton, setShowSelectionButton] = useState(false);
  const [selectionPosition, setSelectionPosition] = useState({ x: 0, y: 0 });

  // Auto-scroll to bottom
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Welcome message
  useEffect(() => {
    if (messages.length === 0) {
      setMessages([{
        id: 'welcome',
        role: 'assistant',
        content: 'Hey there! 👋 I\'m your friendly AI tutor for Physical AI & Humanoid Robotics. Whether you\'re curious about ROS 2, simulation, NVIDIA Isaac, or VLA models - I\'m here to help you learn! What would you like to explore today?',
        timestamp: new Date()
      }]);
    }
  }, []);

  // Handle text selection
  useEffect(() => {
    const handleTextSelection = (e: MouseEvent | TouchEvent) => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      // Don't show button if selection is inside chatbot
      const target = e.target as HTMLElement;
      const isChatbotElement = target.closest('.chatbot-container') || target.closest('.chatbot-toggle');

      if (text && text.length > 0 && !isChatbotElement) {
        setSelectedText(text);

        // Get selection position
        const range = selection?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();

        if (rect) {
          setSelectionPosition({
            x: rect.left + rect.width / 2,
            y: rect.top - 10
          });
          setShowSelectionButton(true);
        }
      } else {
        setShowSelectionButton(false);
      }
    };

    const handleClickOutside = () => {
      // Hide button when clicking outside
      setShowSelectionButton(false);
    };

    // Add event listeners
    document.addEventListener('mouseup', handleTextSelection as EventListener);
    document.addEventListener('touchend', handleTextSelection as EventListener);
    document.addEventListener('mousedown', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleTextSelection as EventListener);
      document.removeEventListener('touchend', handleTextSelection as EventListener);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  const sendMessage = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage: Message = {
      id: `user-${Date.now()}`,
      role: 'user',
      content: input.trim(),
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await fetch(getApiUrl('/api/chat/message'), {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: userMessage.content,
          session_id: sessionId,
          user_id: null
        })
      });

      if (!response.ok) {
        // Try to get error details from response
        let errorDetail = `HTTP error! status: ${response.status}`;
        try {
          const errorData = await response.json();
          errorDetail = errorData.detail || errorData.message || errorDetail;
        } catch {
          // If response is not JSON, use status text
          errorDetail = `HTTP ${response.status}: ${response.statusText}`;
        }
        throw new Error(errorDetail);
      }

      const data = await response.json();

      const assistantMessage: Message = {
        id: `assistant-${Date.now()}`,
        role: 'assistant',
        content: data.response,
        citations: data.citations,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, assistantMessage]);

    } catch (error) {
      console.error('Chat error:', error);
      
      // Provide more detailed error message
      let errorContent = 'Sorry, I encountered an error. Please try again.';
      
      if (error instanceof Error) {
        if (error.message.includes('Failed to fetch') || error.message.includes('NetworkError')) {
          errorContent = 'Unable to connect to the server. Please make sure the backend is running on http://localhost:8000';
        } else if (error.message.includes('HTTP error')) {
          errorContent = `Server error: ${error.message}. Please check the backend logs.`;
        } else {
          errorContent = `Error: ${error.message}`;
        }
      }

      const errorMessage: Message = {
        id: `error-${Date.now()}`,
        role: 'assistant',
        content: errorContent,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const handleAskAboutSelection = () => {
    // Open chatbot and set input with selected text context
    setIsOpen(true);
    setInput(`Explain this: "${selectedText}"`);
    setShowSelectionButton(false);

    // Scroll to input area
    setTimeout(() => {
      const textarea = document.querySelector('.chatbot-input textarea') as HTMLTextAreaElement;
      textarea?.focus();
    }, 100);
  };

  return (
    <>
      {/* Text Selection Quick Chat Button */}
      {showSelectionButton && (
        <button
          className="text-selection-button"
          onClick={handleAskAboutSelection}
          style={{
            position: 'fixed',
            left: `${selectionPosition.x}px`,
            top: `${selectionPosition.y}px`,
            transform: 'translate(-50%, -100%)',
            width: '40px',
            height: '40px',
            borderRadius: '50%',
            border: 'none',
            background: 'var(--ifm-color-primary)',
            color: 'white',
            fontSize: '18px',
            cursor: 'pointer',
            boxShadow: '0 4px 12px rgba(0,0,0,0.25)',
            zIndex: 1001,
            transition: 'all 0.2s',
          }}
          title="Ask AI about this text"
        >
          💬
        </button>
      )}

      {/* Floating Chat Button */}
      <button
        className="chatbot-toggle"
        onClick={() => setIsOpen(!isOpen)}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          border: 'none',
          background: 'var(--ifm-color-primary)',
          color: 'white',
          fontSize: '24px',
          cursor: 'pointer',
          boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
          zIndex: 1000,
        }}
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className="chatbot-container">
          <div className="chatbot-header">
            <h3>AI Tutor</h3>
            <button onClick={() => setIsOpen(false)}>✕</button>
          </div>

          <div className="chatbot-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`chatbot-message ${message.role}`}
              >
                <div className="message-content">
                  {message.content}

                  {message.citations && message.citations.length > 0 && (
                    <div className="citations">
                      <strong>Sources:</strong>
                      {message.citations.map((c, i) => (
                        <div key={i}>{c.module} - {c.chapter}</div>
                      ))}
                    </div>
                  )}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className="chatbot-message assistant">
                <div className="thinking-indicator">
                  <span className="thinking-icon">🧠</span>
                  <div className="thinking-dots">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <div className="chatbot-input">
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about ROS 2, simulation, Isaac, or VLA..."
              disabled={isLoading}
              rows={2}
            />
            <button
              onClick={sendMessage}
              disabled={!input.trim() || isLoading}
            >
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
}
