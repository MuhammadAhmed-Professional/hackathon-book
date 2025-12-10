# ADR-003: RAG Implementation Strategy and Chunking Approach

**Status**: Accepted  
**Date**: 2025-01-27  
**Context**: Need to implement RAG (Retrieval-Augmented Generation) for answering questions about the book content. Must decide on chunking strategy, embedding model, vector database, and retrieval approach.

## Decision

We will use:
- **Chunking**: Semantic chunking with 200-300 tokens per chunk, 50 token overlap using tiktoken
- **Embedding Model**: OpenAI text-embedding-3-small (1536 dimensions)
- **Vector Database**: Qdrant Cloud Free Tier
- **Retrieval**: Top 5 chunks using cosine similarity
- **Generation**: OpenAI ChatKit SDK with retrieved chunks as context

## Rationale

### Chunking Strategy
1. **Semantic Chunking (200-300 tokens)**: Balances context preservation with retrieval precision
2. **50 Token Overlap**: Ensures important information at chunk boundaries isn't lost
3. **tiktoken**: Accurate token counting for OpenAI models

### Embedding Model
1. **text-embedding-3-small**: Good balance of quality and cost
2. **1536 dimensions**: Sufficient for semantic similarity while keeping storage manageable
3. **OpenAI consistency**: Same provider for embeddings and chat completions

### Vector Database
1. **Qdrant Cloud Free Tier**: Sufficient for hackathon scale (hundreds of chunks)
2. **Managed Service**: No infrastructure setup required
3. **Fast Similarity Search**: Optimized for vector search operations

### Retrieval Strategy
1. **Top 5 Chunks**: Provides enough context without overwhelming the model
2. **Cosine Similarity**: Standard metric for semantic similarity
3. **Context Window**: Fits within OpenAI model context limits

## Consequences

**Positive**:
- Good balance of retrieval quality and cost
- Simple implementation with managed services
- Scalable to larger book content if needed

**Negative**:
- Qdrant Free Tier has limits (may need upgrade for production)
- Fixed chunk size may split some concepts
- Top 5 may miss relevant chunks in some cases

## Alternatives Considered

- **Smaller chunks (100-150 tokens)**: More precise but may lose context
- **Larger chunks (500+ tokens)**: More context but less precise retrieval
- **Pinecone/Weaviate**: Alternative vector databases, but Qdrant Free Tier is sufficient
- **text-embedding-3-large**: Better quality but higher cost

## References

- OpenAI Embeddings Guide: https://platform.openai.com/docs/guides/embeddings
- Qdrant Documentation: https://qdrant.tech/documentation/
- Semantic Chunking Best Practices: https://www.pinecone.io/learn/chunking-strategies/

