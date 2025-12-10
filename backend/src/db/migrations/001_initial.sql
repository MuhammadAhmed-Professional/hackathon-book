-- Migration 001: Initial Schema for Physical AI Textbook
-- Feature: 002-physical-ai-book
-- Date: 2025-11-28
-- Purpose: Create users and conversations tables for Better-auth and conversation persistence

-- Users table: Store user accounts with authentication credentials and background information
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    software_background JSONB,
    hardware_background JSONB,
    created_at TIMESTAMP DEFAULT NOW(),
    last_login TIMESTAMP
);

-- Indexes for users table
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_created_at ON users(created_at DESC);

-- Conversations table: Store all chatbot interactions with user attribution and source citations
CREATE TABLE conversations (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE SET NULL,
    session_id VARCHAR(255),
    question TEXT NOT NULL,
    answer TEXT NOT NULL,
    citations JSONB,
    question_type VARCHAR(50) CHECK (question_type IN ('rag', 'selected_text')),
    timestamp TIMESTAMP DEFAULT NOW()
);

-- Indexes for conversations table
CREATE INDEX idx_conversations_user_id ON conversations(user_id);
CREATE INDEX idx_conversations_session_id ON conversations(session_id);
CREATE INDEX idx_conversations_timestamp ON conversations(timestamp DESC);
CREATE INDEX idx_conversations_question_type ON conversations(question_type);

-- Verify tables created
SELECT table_name FROM information_schema.tables
WHERE table_schema = 'public' AND table_name IN ('users', 'conversations');
