import React, { useState, useEffect } from 'react';
import styles from './PersonalizeButton.module.css';

interface PersonalizeButtonProps {
  originalContent: string;
  chapterId: string;
  onContentChange: (content: string, isPersonalized: boolean) => void;
}

interface UserProfile {
  software_experience: string;
  hardware_experience: string;
  name?: string;
}

const PersonalizeButton: React.FC<PersonalizeButtonProps> = ({
  originalContent,
  chapterId,
  onContentChange,
}) => {
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [userProfile, setUserProfile] = useState<UserProfile | null>(null);

  // Load user profile from localStorage on mount
  useEffect(() => {
    const profile = getUserProfile();
    setUserProfile(profile);
  }, []);

  const handlePersonalize = async () => {
    if (isPersonalized) {
      // Reset to original
      setIsPersonalized(false);
      onContentChange(originalContent, false);
      return;
    }

    // Check if user is logged in
    if (!userProfile) {
      setError('Please sign in to personalize content based on your background.');
      return;
    }

    // Check if already personalized (cached)
    if (personalizedContent) {
      setIsPersonalized(true);
      onContentChange(personalizedContent, true);
      return;
    }

    // Personalize content
    setIsLoading(true);
    setError(null);

    try {
      const sessionId = getSessionId();
      const userId = localStorage.getItem('user_id');

      const response = await fetch('http://localhost:8000/api/content/personalize', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          markdown_content: originalContent,
          chapter_id: chapterId,
          user_id: userId,
          software_experience: userProfile.software_experience,
          hardware_experience: userProfile.hardware_experience,
          session_id: sessionId,
        }),
      });

      if (!response.ok) {
        if (response.status === 429) {
          throw new Error('Personalization rate limit reached. Please wait a moment.');
        }
        throw new Error(`Personalization failed: ${response.statusText}`);
      }

      const data = await response.json();
      setPersonalizedContent(data.personalized_content);
      setIsPersonalized(true);
      onContentChange(data.personalized_content, true);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Personalization failed');
      console.error('Personalization error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  // Don't show button if user not logged in
  if (!userProfile) {
    return null;
  }

  return (
    <div className={styles.personalizeButtonContainer}>
      <button
        onClick={handlePersonalize}
        disabled={isLoading}
        className={styles.personalizeButton}
        title={
          isPersonalized
            ? 'Show Original Content'
            : `Personalize for ${userProfile.software_experience} software / ${userProfile.hardware_experience} hardware`
        }
      >
        {isLoading ? (
          <>
            <span className={styles.spinner}></span>
            Personalizing...
          </>
        ) : isPersonalized ? (
          <>🔄 Show Original</>
        ) : (
          <>✨ Personalize for You</>
        )}
      </button>
      {userProfile && !isPersonalized && (
        <span className={styles.profileHint}>
          Adapted for: {formatExperience(userProfile.software_experience)} software,{' '}
          {formatExperience(userProfile.hardware_experience)} hardware
        </span>
      )}
      {error && <div className={styles.error}>{error}</div>}
    </div>
  );
};

// Helper functions
function getUserProfile(): UserProfile | null {
  const profileStr = localStorage.getItem('user_profile');
  if (!profileStr) {
    return null;
  }
  try {
    return JSON.parse(profileStr);
  } catch {
    return null;
  }
}

function getSessionId(): string {
  let sessionId = localStorage.getItem('session_id');
  if (!sessionId) {
    sessionId = 'session_' + Math.random().toString(36).substring(2, 15);
    localStorage.setItem('session_id', sessionId);
  }
  return sessionId;
}

function formatExperience(level: string): string {
  return level.charAt(0).toUpperCase() + level.slice(1);
}

export default PersonalizeButton;
