# API Contracts

This directory contains API contract specifications for the Hackathon App backend.

## Files

- `openapi.yaml`: OpenAPI 3.0.3 specification for the FastAPI backend

## Endpoints

### Health Check
- `GET /ping`: Health check endpoint

### Ingestion
- `POST /ingest`: Ingest book content into Qdrant vector database

### Chatbot
- `POST /ask`: Ask questions using RAG (Retrieval-Augmented Generation)
- `POST /ask_selected`: Ask questions about selected text (no RAG query)

## Usage

### View API Documentation

FastAPI automatically generates interactive API documentation from this OpenAPI spec:

- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

### Validate Contracts

Use tools like `openapi-spec-validator` to validate the specification:

```bash
pip install openapi-spec-validator
openapi-spec-validator contracts/openapi.yaml
```

### Generate Client Code

Use OpenAPI generators to create client code for frontend:

```bash
npx @openapitools/openapi-generator-cli generate \
  -i contracts/openapi.yaml \
  -g typescript-axios \
  -o frontend/src/api
```

## Contract Testing

Contract tests should verify that the FastAPI implementation matches this specification. See `backend/tests/contract/` for contract test implementations.

