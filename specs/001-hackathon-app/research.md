# Research: Hackathon App - Docusaurus Book with RAG Chatbot

**Date**: 2025-01-27  
**Feature**: 001-hackathon-app  
**Purpose**: Resolve technical unknowns and document technology choices

## Technology Choices

### Docusaurus for Documentation Site

**Decision**: Use Docusaurus v3.x (latest stable) for the book/documentation site

**Rationale**: 
- Docusaurus is specifically designed for documentation sites with excellent markdown support
- Built-in support for GitHub Pages deployment
- React-based, allowing easy integration of custom chatbot components
- Excellent navigation and internal linking capabilities
- Active community and comprehensive documentation
- Free and open-source

**Alternatives considered**:
- **GitBook**: Commercial product, less flexible for custom components
- **MkDocs**: Python-based but less React-friendly for chatbot integration
- **VitePress**: Vue-based, but team is more familiar with React ecosystem
- **Custom React site**: Too much boilerplate for a documentation-focused project

**Implementation notes**:
- Use Docusaurus classic template for maximum compatibility
- Configure for GitHub Pages deployment (baseUrl, organizationName, projectName)
- Enable React components in markdown via MDX

### FastAPI for Backend

**Decision**: Use FastAPI with Python 3.11+

**Rationale**:
- Modern, fast Python web framework with automatic API documentation
- Excellent async support for concurrent requests
- Built-in OpenAPI/Swagger documentation
- Easy integration with OpenAI SDKs
- Simple deployment options
- Strong typing support with Pydantic

**Alternatives considered**:
- **Flask**: Older framework, less modern async support, more boilerplate
- **Django**: Over-engineered for a simple API, heavier dependencies
- **Express.js (Node.js)**: Team preference for Python ecosystem

**Implementation notes**:
- Use uvicorn as ASGI server
- Implement CORS middleware for frontend-backend communication
- Use Pydantic models for request/response validation

### Qdrant Cloud Free Tier for Vector Database

**Decision**: Use Qdrant Cloud Free Tier for vector storage and similarity search

**Rationale**:
- Managed service eliminates infrastructure management
- Free tier sufficient for hackathon scale (1 collection, reasonable size limits)
- Excellent Python client library
- Fast similarity search performance
- Good documentation and community support

**Alternatives considered**:
- **Pinecone**: Free tier available but more restrictive, commercial focus
- **Weaviate Cloud**: More complex setup, overkill for this use case
- **ChromaDB (local)**: Would require hosting infrastructure, not suitable for GitHub Pages deployment
- **FAISS (local)**: Requires file storage and hosting, adds complexity

**Implementation notes**:
- Create single collection for book chunks
- Use OpenAI embeddings (text-embedding-3-small or text-embedding-ada-002)
- Store metadata (chapter, chunk_id, source) alongside vectors
- Implement retry logic for API rate limits

### OpenAI SDKs (Agents SDK + ChatKit SDK)

**Decision**: Use both OpenAI Agents SDK and OpenAI ChatKit SDK as required by hackathon specifications

**Rationale**:
- Hackathon requirement explicitly states both SDKs must be used
- Agents SDK provides structured agent capabilities
- ChatKit SDK provides chat completion functionality
- Both SDKs are official OpenAI offerings with good documentation

**Alternatives considered**:
- **Standard OpenAI Python library only**: Does not meet hackathon requirement for both SDKs
- **Custom API calls**: Less structured, more error-prone

**Implementation notes**:
- Use Agents SDK for agent skills (SummarizeSection, GenerateQuizQuestions, ExplainTerm)
- Use ChatKit SDK for RAG chat completions
- Use standard OpenAI library for embeddings (text-embedding models)
- Ensure proper error handling for both SDKs

### Text Chunking Strategy

**Decision**: Use semantic chunking with overlap (200-300 tokens per chunk, 50 token overlap)

**Rationale**:
- Semantic chunking preserves context better than fixed-size splitting
- Overlap ensures no information is lost at chunk boundaries
- 200-300 tokens balances context preservation with retrieval precision
- Works well with OpenAI embedding models

**Alternatives considered**:
- **Fixed-size chunking**: Simpler but loses context at boundaries
- **Sentence-based chunking**: More complex, may create very small chunks
- **Paragraph-based chunking**: May create chunks too large for effective retrieval

**Implementation notes**:
- Use tiktoken library for accurate token counting
- Preserve chapter metadata with each chunk
- Store chunk text, embedding, chapter reference, and chunk index

### React Component Architecture

**Decision**: Create separate React components for chatbot and selected-text handler, embed in Docusaurus via MDX

**Rationale**:
- Separation of concerns: chatbot logic separate from text selection logic
- Reusable components that can be embedded in any Docusaurus page
- MDX allows React components in markdown content
- Maintains Docusaurus structure while adding interactivity

**Alternatives considered**:
- **Single monolithic component**: Less maintainable, harder to test
- **Iframe embedding**: Adds complexity, communication overhead
- **External widget**: Requires separate hosting, adds deployment complexity

**Implementation notes**:
- Create Chatbot.tsx component with chat interface
- Create SelectedTextHandler.tsx for text selection and context passing
- Use React hooks for state management
- Implement proper error handling and loading states

### Deployment Strategy

**Decision**: 
- Frontend: GitHub Pages (via Docusaurus deployment)
- Backend: Local development server or cloud hosting (Render, Fly.io, Railway) for demo

**Rationale**:
- GitHub Pages is free and integrates with Docusaurus
- Backend can run locally for development/demo
- Cloud hosting options available if needed for public access
- Environment variables work in both scenarios

**Alternatives considered**:
- **Vercel/Netlify for frontend**: GitHub Pages is simpler and free for public repos
- **AWS/GCP for backend**: Overkill for hackathon, adds complexity
- **Docker containers**: Adds deployment complexity, not necessary for hackathon scale

**Implementation notes**:
- Configure Docusaurus for GitHub Pages (baseUrl, trailingSlash)
- Use GitHub Actions for automated deployment (optional)
- Document local backend setup in quickstart.md
- Provide .env.example template

## Integration Patterns

### Frontend-Backend Communication

**Decision**: RESTful API with JSON, CORS enabled

**Rationale**:
- Standard, well-understood pattern
- Easy to test and debug
- Works with FastAPI automatic OpenAPI docs
- Simple error handling

**Implementation notes**:
- FastAPI CORS middleware configured for frontend origin
- JSON request/response format
- Standard HTTP status codes
- Error responses in consistent format

### RAG Query Flow

**Decision**: 
1. User question → OpenAI embedding
2. Embedding → Qdrant similarity search (top 5 chunks)
3. Retrieved chunks + question → OpenAI ChatKit SDK completion
4. Response + sources → Frontend display

**Rationale**:
- Standard RAG pattern proven in production
- Top 5 chunks provide sufficient context without overwhelming the model
- ChatKit SDK handles chat completion with context injection
- Source citations enable transparency

**Implementation notes**:
- Use cosine similarity for vector search
- Filter by relevance score threshold (e.g., > 0.7)
- Include chunk metadata in response for citations
- Handle empty results gracefully

## Best Practices Adopted

1. **Environment-based configuration**: All secrets in .env files, never committed
2. **Error handling**: User-friendly messages, proper HTTP status codes
3. **Logging**: Structured logging for debugging without exposing secrets
4. **Code organization**: Clear separation of models, services, and API routes
5. **Documentation**: Inline comments, docstrings, and external documentation
6. **Testing**: Unit tests for services, integration tests for API endpoints

## Unresolved Items

None - all technical decisions resolved based on hackathon requirements and best practices.

