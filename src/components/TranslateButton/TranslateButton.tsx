import React, { useState } from 'react';
import styles from './TranslateButton.module.css';

interface TranslateButtonProps {
  originalContent: string;
  chapterId?: string;
  onContentChange: (content: string, isUrdu: boolean) => void;
}

const TranslateButton: React.FC<TranslateButtonProps> = ({
  originalContent,
  chapterId,
  onContentChange,
}) => {
  const [isUrdu, setIsUrdu] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);

  const handleTranslate = async () => {
    if (isUrdu) {
      // Switch back to English
      setIsUrdu(false);
      onContentChange(originalContent, false);
      return;
    }

    // Check if already translated (cached)
    if (translatedContent) {
      setIsUrdu(true);
      onContentChange(translatedContent, true);
      return;
    }

    // Translate to Urdu
    setIsLoading(true);
    setError(null);

    try {
      const sessionId = getSessionId();
      const response = await fetch('http://localhost:8000/api/content/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          markdown_content: originalContent,
          chapter_id: chapterId,
          session_id: sessionId,
        }),
      });

      if (!response.ok) {
        if (response.status === 429) {
          throw new Error('Translation rate limit reached. Please wait a moment.');
        }
        throw new Error(`Translation failed: ${response.statusText}`);
      }

      const data = await response.json();
      setTranslatedContent(data.translated_content);
      setIsUrdu(true);
      onContentChange(data.translated_content, true);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Translation failed');
      console.error('Translation error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.translateButtonContainer}>
      <button
        onClick={handleTranslate}
        disabled={isLoading}
        className={styles.translateButton}
        title={isUrdu ? 'Show Original English' : 'Translate to Urdu'}
      >
        {isLoading ? (
          <>
            <span className={styles.spinner}></span>
            Translating...
          </>
        ) : isUrdu ? (
          <>🇬🇧 Show Original English</>
        ) : (
          <>🇵🇰 Translate to Urdu</>
        )}
      </button>
      {error && <div className={styles.error}>{error}</div>}
    </div>
  );
};

// Helper function to get or create session ID
function getSessionId(): string {
  let sessionId = localStorage.getItem('session_id');
  if (!sessionId) {
    sessionId = 'session_' + Math.random().toString(36).substring(2, 15);
    localStorage.setItem('session_id', sessionId);
  }
  return sessionId;
}

export default TranslateButton;
