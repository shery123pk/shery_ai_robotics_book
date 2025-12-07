-- Migration: 001_create_users_table.sql
-- Description: Create users table for authentication and user profiles
-- Created: 2025-12-06

-- Create users table
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    software_experience VARCHAR(50) NOT NULL CHECK (software_experience IN ('beginner', 'intermediate', 'advanced')),
    hardware_experience VARCHAR(50) NOT NULL CHECK (hardware_experience IN ('none', 'hobbyist', 'professional')),
    created_at TIMESTAMP DEFAULT NOW(),
    last_login TIMESTAMP
);

-- Create index on email for fast lookup
CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);

-- Add comment
COMMENT ON TABLE users IS 'User accounts with authentication and background questionnaire data';
COMMENT ON COLUMN users.software_experience IS 'Software development experience level: beginner, intermediate, or advanced';
COMMENT ON COLUMN users.hardware_experience IS 'Hardware/robotics experience level: none, hobbyist, or professional';
