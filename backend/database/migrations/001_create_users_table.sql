-- Migration: 001_create_users_table.sql
-- Description: Create users table for authentication and user profiles
-- Created: 2025-12-06

-- Drop table if exists to recreate with correct schema
DROP TABLE IF EXISTS users CASCADE;

-- Create users table
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    full_name VARCHAR(100) NOT NULL,
    role VARCHAR(50) DEFAULT 'student',
    background TEXT,
    preferences JSONB DEFAULT '{}',
    created_at TIMESTAMP DEFAULT NOW(),
    last_login TIMESTAMP
);

-- Create index on email for fast lookup
CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);

-- Add comment
COMMENT ON TABLE users IS 'User accounts with authentication and user profiles';
COMMENT ON COLUMN users.full_name IS 'User''s full name';
COMMENT ON COLUMN users.role IS 'User role (student, instructor, researcher)';
COMMENT ON COLUMN users.background IS 'Educational or professional background';
