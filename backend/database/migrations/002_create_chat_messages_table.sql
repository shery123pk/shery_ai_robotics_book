-- Migration: 002_create_chat_messages_table.sql
-- Description: Create chat_messages table for chatbot conversation history
-- Created: 2025-12-06

-- Create chat_messages table
CREATE TABLE IF NOT EXISTS chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,  -- Nullable for anonymous users
    session_id VARCHAR(255) NOT NULL,  -- Browser session ID
    message_text TEXT NOT NULL,
    response_text TEXT NOT NULL,
    citations JSONB,  -- Array of {module, chapter, section}
    created_at TIMESTAMP DEFAULT NOW()
);

-- Create indexes for efficient queries
CREATE INDEX IF NOT EXISTS idx_chat_user_session ON chat_messages(user_id, session_id);
CREATE INDEX IF NOT EXISTS idx_chat_session ON chat_messages(session_id);
CREATE INDEX IF NOT EXISTS idx_chat_created_at ON chat_messages(created_at DESC);

-- Add comments
COMMENT ON TABLE chat_messages IS 'Chatbot conversation history for logged-in and anonymous users';
COMMENT ON COLUMN chat_messages.user_id IS 'User ID (nullable for anonymous sessions)';
COMMENT ON COLUMN chat_messages.session_id IS 'Browser session UUID for anonymous users';
COMMENT ON COLUMN chat_messages.citations IS 'JSON array of citation objects with module, chapter, section';
