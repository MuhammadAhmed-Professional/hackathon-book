# Data Model: Hackathon App - Docusaurus Book with RAG Chatbot

**Date**: 2025-01-27  
**Feature**: 001-hackathon-app

## Entities

### BookChapter

Represents a chapter in the Docusaurus book.

**Attributes**:
- `id` (string): Unique identifier (e.g., "intro", "ai-driven-development")
- `title` (string): Chapter title displayed in navigation
- `content` (string): Markdown content of the chapter
- `order` (integer): Display order in navigation/sidebar
- `path` (string): File path relative to docs/ directory (e.g., "intro.md")
- `internal_links` (array of strings): List of chapter IDs this chapter links to

**Validation Rules**:
- `id` must be unique across all chapters
- `title` must be non-empty
- `content` must be valid markdown
- `order` must be positive integer
- `path` must exist in docs/ directory

**Relationships**:
- One-to-many with TextChunk (a chapter contains multiple chunks)

### TextChunk

Represents a piece of book content stored in the vector database.

**Attributes**:
- `chunk_id` (string): Unique identifier for the chunk
- `text` (string): The chunk text content (200-300 tokens)
- `embedding` (array of floats): Vector embedding from OpenAI (1536 dimensions for text-embedding-3-small)
- `chapter_id` (string): Reference to source BookChapter
- `chunk_index` (integer): Position of chunk within chapter
- `token_count` (integer): Number of tokens in the chunk
- `metadata` (object): Additional metadata (e.g., section title, page number if applicable)

**Validation Rules**:
- `chunk_id` must be unique
- `text` must be non-empty and between 50-500 tokens
- `embedding` must be valid vector (1536 dimensions)
- `chapter_id` must reference existing BookChapter
- `chunk_index` must be non-negative

**Relationships**:
- Many-to-one with BookChapter (chunks belong to a chapter)
- Referenced by Answer entities via source citations

**Storage**: Stored in Qdrant Cloud collection with vector index on `embedding` field

### Question

Represents a user's query about the book.

**Attributes**:
- `question_id` (string, optional): Unique identifier (generated if needed)
- `question_text` (string): The user's question
- `selected_text` (string, optional): Selected text context (for selected-text mode)
- `mode` (enum): "rag" or "selected" - determines query type
- `timestamp` (datetime): When the question was asked
- `session_id` (string, optional): User session identifier for conversation tracking

**Validation Rules**:
- `question_text` must be non-empty and under 1000 characters
- `selected_text` must be non-empty if `mode` is "selected"
- `mode` must be either "rag" or "selected"
- `timestamp` must be valid datetime

**Relationships**:
- One-to-one with Answer (each question produces one answer)

### Answer

Represents the chatbot's response to a question.

**Attributes**:
- `answer_id` (string): Unique identifier
- `answer_text` (string): The generated answer
- `sources` (array of SourceCitation): List of source chunks used
- `confidence` (float, optional): Confidence score (0.0-1.0) if available
- `question_id` (string): Reference to the Question
- `timestamp` (datetime): When the answer was generated
- `model_used` (string): OpenAI model identifier (e.g., "gpt-4", "gpt-3.5-turbo")
- `error_message` (string, optional): Error message if generation failed

**Validation Rules**:
- `answer_id` must be unique
- `answer_text` must be non-empty (unless error occurred)
- `sources` array may be empty if no relevant chunks found
- `confidence` must be between 0.0 and 1.0 if provided
- `question_id` must reference existing Question

**Relationships**:
- One-to-one with Question
- Many-to-many with TextChunk (via sources array)

### SourceCitation

Represents a reference to a source chunk used in an answer.

**Attributes**:
- `chunk_id` (string): Reference to TextChunk
- `chapter_id` (string): Reference to BookChapter (for display)
- `relevance_score` (float): Similarity score from vector search (0.0-1.0)
- `snippet` (string, optional): Preview text from the chunk

**Validation Rules**:
- `chunk_id` must reference existing TextChunk
- `relevance_score` must be between 0.0 and 1.0
- `snippet` should be first 100-200 characters of chunk text

### AgentSkill

Represents a reusable agent skill/subagent.

**Attributes**:
- `skill_id` (string): Unique identifier (e.g., "summarize-section", "generate-quiz")
- `skill_name` (string): Human-readable name
- `persona` (string): Persona description for the skill
- `questions` (array of strings): Analytical questions the skill asks
- `principles` (array of strings): Decision principles the skill follows
- `input_schema` (object): JSON schema for input validation
- `output_schema` (object): JSON schema for output validation

**Validation Rules**:
- `skill_id` must be unique and follow naming convention
- `persona` must be non-empty and descriptive
- `questions` array must have at least 3 questions
- `principles` array must have at least 2 principles
- `input_schema` and `output_schema` must be valid JSON schemas

**Relationships**:
- Independent entity (skills are reusable across projects)

**Skills Defined**:
1. **SummarizeSection**: Takes text, returns concise summary
2. **GenerateQuizQuestions**: Takes text, returns multiple-choice questions
3. **ExplainTerm**: Takes term, returns explanation using book context

## State Transitions

### Question Processing Flow

```
Question Created
    ↓
[If mode="rag"]
    Embed Question → Query Qdrant → Retrieve Chunks → Generate Answer
[If mode="selected"]
    Use Selected Text → Generate Answer (no RAG query)
    ↓
Answer Generated (or Error)
    ↓
Answer Returned to User
```

### Chunk Ingestion Flow

```
Book Chapter Content
    ↓
Split into Chunks (200-300 tokens, 50 token overlap)
    ↓
Generate Embeddings (OpenAI)
    ↓
Store in Qdrant (chunk + embedding + metadata)
    ↓
Ingestion Complete
```

## Data Storage

### Qdrant Cloud Collection Schema

**Collection Name**: `book_chunks`

**Vector Configuration**:
- Vector size: 1536 (OpenAI text-embedding-3-small)
- Distance metric: Cosine

**Payload Schema**:
```json
{
  "chunk_id": "string",
  "text": "string",
  "chapter_id": "string",
  "chunk_index": "integer",
  "token_count": "integer",
  "metadata": {
    "section_title": "string (optional)",
    "created_at": "datetime"
  }
}
```

### Local File Storage (Docusaurus)

**Structure**:
```
frontend/docs/
├── intro.md
├── ai-driven-development.md
├── spec-driven-development.md
├── rag-fundamentals.md
├── implementation-guide.md
├── chatbot-usage.md
└── future-work.md
```

Each file contains markdown content with frontmatter:
```yaml
---
id: chapter-id
title: Chapter Title
sidebar_position: 1
---
```

## Validation Rules Summary

- All string fields have maximum length limits to prevent abuse
- All IDs must be unique within their entity type
- All references (foreign keys) must point to existing entities
- All timestamps must be valid datetime values
- All numeric scores must be within defined ranges (0.0-1.0)
- All required fields must be non-empty
- All enum values must be from allowed set

