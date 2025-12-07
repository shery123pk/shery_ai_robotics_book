-- Migration: 003_create_personalized_content_table.sql
-- Description: Create personalized_content table for caching user-specific chapter adaptations
-- Created: 2025-12-06

-- Create personalized_content table
CREATE TABLE IF NOT EXISTS personalized_content (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    chapter_id VARCHAR(100) NOT NULL,  -- e.g., 'module-1/02-nodes-topics'
    original_markdown TEXT NOT NULL,
    personalized_markdown TEXT NOT NULL,
    generated_at TIMESTAMP DEFAULT NOW(),
    UNIQUE(user_id, chapter_id)  -- One personalized version per user per chapter
);

-- Create index for fast lookup
CREATE INDEX IF NOT EXISTS idx_personalized_user_chapter ON personalized_content(user_id, chapter_id);
CREATE INDEX IF NOT EXISTS idx_personalized_generated_at ON personalized_content(generated_at DESC);

-- Add comments
COMMENT ON TABLE personalized_content IS 'Cached personalized chapter content adapted to user background';
COMMENT ON COLUMN personalized_content.chapter_id IS 'Chapter identifier (e.g., module-1/02-nodes-topics)';
COMMENT ON COLUMN personalized_content.original_markdown IS 'Original chapter markdown for comparison';
COMMENT ON COLUMN personalized_content.personalized_markdown IS 'Adapted markdown based on user software/hardware experience';
COMMENT ON CONSTRAINT personalized_content_user_id_chapter_id_key ON personalized_content IS 'Ensure one personalized version per user per chapter';
