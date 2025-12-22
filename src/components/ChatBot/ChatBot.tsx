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
  const [showAuthorInfo, setShowAuthorInfo] = useState(false);

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
        content: 'Hey there! 👋 I\'m your friendly AI tutor for Physical AI & Humanoid Robotics, created by Sharmeen Asif. Whether you\'re curious about ROS 2, simulation, NVIDIA Isaac, or VLA models - I\'m here to help you learn!\n\nClick "👤 About Author" in the header to learn more about the creator, or ask me anything about robotics!',
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

    const handleClickOutside = (e: MouseEvent) => {
      // Don't hide button if clicking the selection button itself
      const target = e.target as HTMLElement;
      if (target.closest('.text-selection-button')) {
        return;
      }
      // Hide button when clicking outside
      setShowSelectionButton(false);
    };

    // Add event listeners
    document.addEventListener('mouseup', handleTextSelection as EventListener);
    document.addEventListener('touchend', handleTextSelection as EventListener);
    document.addEventListener('mousedown', handleClickOutside as EventListener);

    return () => {
      document.removeEventListener('mouseup', handleTextSelection as EventListener);
      document.removeEventListener('touchend', handleTextSelection as EventListener);
      document.removeEventListener('mousedown', handleClickOutside as EventListener);
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

  const clearChat = () => {
    if (window.confirm('Are you sure you want to clear all chat messages?')) {
      // Keep only the welcome message
      setMessages([{
        id: 'welcome',
        role: 'assistant',
        content: 'Hey there! 👋 I\'m your friendly AI tutor for Physical AI & Humanoid Robotics, created by Sharmeen Asif. Whether you\'re curious about ROS 2, simulation, NVIDIA Isaac, or VLA models - I\'m here to help you learn!\n\nClick "👤 About Author" in the header to learn more about the creator, or ask me anything about robotics!',
        timestamp: new Date()
      }]);
    }
  };

  const showAuthorProfile = () => {
    const authorMessage: Message = {
      id: `author-${Date.now()}`,
      role: 'assistant',
      content: `📚 **About the Author - Sharmeen Asif**\n\n**Location:** Karachi, Pakistan\n**Contact:** codeshery@gmail.com | +92 321 2783184\n**LinkedIn:** linkedin.com/in/sharmeen-asif-654727373\n**GitHub:** github.com/shery123pk\n\n**Professional Summary:**\nIT Administrative Professional with 18+ years of experience in computer operations, technical training, and data management. Currently advancing skills in AI, Python, and Next.js through PIAIC and GIAIC programs.\n\n**Education:**\n• Master of Computer Science (MCS) - COMSATS University Islamabad (2018)\n• Bachelor of Education (B.Ed.) - AIOU (2014)\n• Diploma in Information Technology - SBTE (2005)\n• BSc Premedical - University of Karachi\n\n**Current Training:**\n• PIAIC Agent AI Program (Level 3) - AI agents & automation\n• GIAIC - AI, ML, and modern web development\n\n**Core Expertise:**\n• Python | Next.js | Agent SDK | n8n | MS Office Suite\n• AI Development | System Administration | Technical Training\n• 18+ years in IT administration and education\n\n**Professional Experience:**\n• EAB Haroon Bahria College (2012-Present) - IT Administrator\n• Pakistan Navy Computer Training Center (2005-2010) - Technical Trainer\n\nSharmeen has trained 200+ students annually and maintains 100% system uptime for 500+ users. This AI-powered textbook is a result of her passion for combining education with cutting-edge AI technology! 🚀`,
      timestamp: new Date()
    };
    setMessages(prev => [...prev, authorMessage]);
    setShowAuthorInfo(false);
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
            width: '48px',
            height: '48px',
            borderRadius: '50%',
            border: '2px solid white',
            background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
            color: 'white',
            fontSize: '20px',
            cursor: 'pointer',
            boxShadow: '0 6px 20px rgba(102, 126, 234, 0.5)',
            zIndex: 1001,
            transition: 'all 0.3s ease',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            animation: 'popIn 0.3s cubic-bezier(0.68, -0.55, 0.265, 1.55)',
          }}
          onMouseEnter={(e) => {
            e.currentTarget.style.transform = 'translate(-50%, -100%) scale(1.1)';
            e.currentTarget.style.boxShadow = '0 8px 24px rgba(102, 126, 234, 0.6)';
          }}
          onMouseLeave={(e) => {
            e.currentTarget.style.transform = 'translate(-50%, -100%) scale(1)';
            e.currentTarget.style.boxShadow = '0 6px 20px rgba(102, 126, 234, 0.5)';
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
          background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
          color: 'white',
          fontSize: '24px',
          cursor: 'pointer',
          boxShadow: '0 6px 20px rgba(102, 126, 234, 0.4)',
          zIndex: 1000,
          transition: 'all 0.3s ease',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.transform = 'scale(1.1)';
          e.currentTarget.style.boxShadow = '0 8px 24px rgba(102, 126, 234, 0.5)';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.transform = 'scale(1)';
          e.currentTarget.style.boxShadow = '0 6px 20px rgba(102, 126, 234, 0.4)';
        }}
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className="chatbot-container">
          <div className="chatbot-header">
            <h3>AI Tutor</h3>
            <div style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
              <button
                onClick={showAuthorProfile}
                style={{
                  background: 'rgba(255, 255, 255, 0.2)',
                  border: 'none',
                  color: 'white',
                  padding: '6px 12px',
                  borderRadius: '16px',
                  cursor: 'pointer',
                  fontSize: '12px',
                  fontWeight: '600',
                  transition: 'all 0.2s',
                  display: 'flex',
                  alignItems: 'center',
                  gap: '4px',
                }}
                onMouseEnter={(e) => {
                  e.currentTarget.style.background = 'rgba(255, 255, 255, 0.3)';
                }}
                onMouseLeave={(e) => {
                  e.currentTarget.style.background = 'rgba(255, 255, 255, 0.2)';
                }}
                title="About the author"
              >
                👤 Author
              </button>
              <button
                onClick={clearChat}
                style={{
                  background: 'rgba(255, 255, 255, 0.2)',
                  border: 'none',
                  color: 'white',
                  padding: '6px 12px',
                  borderRadius: '16px',
                  cursor: 'pointer',
                  fontSize: '12px',
                  fontWeight: '600',
                  transition: 'all 0.2s',
                  display: 'flex',
                  alignItems: 'center',
                  gap: '4px',
                }}
                onMouseEnter={(e) => {
                  e.currentTarget.style.background = 'rgba(255, 255, 255, 0.3)';
                }}
                onMouseLeave={(e) => {
                  e.currentTarget.style.background = 'rgba(255, 255, 255, 0.2)';
                }}
                title="Clear all messages"
              >
                🗑️ Clear
              </button>
              <button onClick={() => setIsOpen(false)}>✕</button>
            </div>
          </div>

          <div className="chatbot-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`chatbot-message ${message.role}`}
              >
                <div className="message-content" style={{ whiteSpace: 'pre-wrap' }}>
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
