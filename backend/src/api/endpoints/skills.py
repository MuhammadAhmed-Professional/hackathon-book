"""
Agent Skills API endpoints.
Feature: 002-physical-ai-book (Phase 9: Agent Skills)
"""
from typing import Optional
from fastapi import APIRouter, HTTPException, Header
from pydantic import BaseModel, Field

from src.services.agent_skills_service import (
    summarize_section,
    generate_quiz_questions,
    explain_term,
    get_available_skills
)
from src.services.auth_service import verify_token

router = APIRouter()


# Request/Response Models

class SummarizeRequest(BaseModel):
    content: str = Field(..., description="Section/chapter content to summarize")
    tier: Optional[str] = Field(None, description="Summary tier: 'tldr', 'standard', 'detailed', or None for all")


class QuizRequest(BaseModel):
    content: str = Field(..., description="Chapter content for quiz generation")
    num_questions: int = Field(10, ge=1, le=20, description="Number of questions (1-20)")
    difficulty: Optional[str] = Field(None, description="Difficulty: 'easy', 'medium', 'hard', or None for mixed")


class ExplainRequest(BaseModel):
    term: str = Field(..., description="Technical term to explain")
    context: Optional[str] = Field(None, description="Context where term appeared")
    tier: Optional[str] = Field(None, description="Explanation tier: 'eli5', 'standard', 'technical', or None for all")


class SkillResponse(BaseModel):
    output: str = Field(..., description="Generated output from the skill")
    model_used: str = Field(..., description="OpenAI model used")
    skill_name: str = Field(..., description="Name of executed skill")


class AvailableSkillsResponse(BaseModel):
    skills: dict = Field(..., description="Dictionary of available skills with descriptions")


# Endpoints

@router.get("/skills", response_model=AvailableSkillsResponse)
async def list_available_skills():
    """
    Get list of available agent skills.

    Returns:
        Dictionary of skill IDs and descriptions
    """
    skills = get_available_skills()
    return AvailableSkillsResponse(skills=skills)


@router.post("/skills/summarize", response_model=SkillResponse)
async def execute_summarize_skill(
    request: SummarizeRequest,
    authorization: Optional[str] = Header(None)
):
    """
    Execute the SummarizeSection skill to generate multi-tier summaries.

    **Skill**: Creates TL;DR (30 words), Standard (60-100 words), and Detailed (200-300 words) summaries.

    **Use Case**: Quick chapter reviews, learning reinforcement, content preview

    **Example**:
    ```json
    {
      "content": "This chapter covers ROS 2 nodes, topics, and services...",
      "tier": "standard"
    }
    ```
    """
    try:
        # Optional: Verify authentication (not required for skills, but available)
        user_tier = "beginner"  # Default
        if authorization and authorization.startswith("Bearer "):
            token = authorization[7:]
            try:
                payload = verify_token(token)
                # Could extract user tier from profile here for personalization
            except:
                pass  # Anonymous users can still use skills

        # Execute skill
        result = summarize_section(
            section_content=request.content,
            tier=request.tier
        )

        return SkillResponse(**result)

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Skill execution failed: {str(e)}")


@router.post("/skills/quiz", response_model=SkillResponse)
async def execute_quiz_skill(
    request: QuizRequest,
    authorization: Optional[str] = Header(None)
):
    """
    Execute the GenerateQuizQuestions skill to create adaptive assessments.

    **Skill**: Generates quiz questions with:
    - Multiple choice, code completion, true/false, short answer types
    - Difficulty tiers (easy, medium, hard) based on Bloom's Taxonomy
    - Plausible distractors that reveal misconceptions
    - Rich feedback explaining correct/incorrect answers

    **Use Case**: Self-assessment, practice questions, formative evaluation

    **Example**:
    ```json
    {
      "content": "Chapter content on ROS 2 publishers and subscribers...",
      "num_questions": 10,
      "difficulty": "medium"
    }
    ```
    """
    try:
        # Optional: Get user tier for adaptive difficulty
        user_tier = "intermediate"  # Default
        if authorization and authorization.startswith("Bearer "):
            token = authorization[7:]
            try:
                payload = verify_token(token)
                # Could extract user tier from profile here
            except:
                pass

        # Execute skill
        result = generate_quiz_questions(
            chapter_content=request.content,
            num_questions=request.num_questions,
            difficulty=request.difficulty
        )

        return SkillResponse(**result)

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Skill execution failed: {str(e)}")


@router.post("/skills/explain", response_model=SkillResponse)
async def execute_explain_skill(
    request: ExplainRequest,
    authorization: Optional[str] = Header(None)
):
    """
    Execute the ExplainTerm skill to provide multi-level explanations.

    **Skill**: Generates explanations at multiple levels:
    - **ELI5**: Simple analogy, no jargon (1-2 sentences)
    - **Standard**: Clear definition with practical context (1 paragraph)
    - **Technical**: Formal definition with implementation details (2-3 paragraphs)

    Includes common misconceptions, related concepts, and code examples.

    **Use Case**: Understanding unfamiliar terms, quick reference, deep technical learning

    **Example**:
    ```json
    {
      "term": "quaternion",
      "context": "Used in geometry_msgs/Pose for orientation representation",
      "tier": "standard"
    }
    ```
    """
    try:
        # Optional: Get user tier to adapt explanation depth
        user_tier = "beginner"  # Default
        if authorization and authorization.startswith("Bearer "):
            token = authorization[7:]
            try:
                payload = verify_token(token)
                # Could extract user tier from profile here
            except:
                pass

        # Execute skill
        result = explain_term(
            term=request.term,
            context=request.context,
            tier=request.tier
        )

        return SkillResponse(**result)

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Skill execution failed: {str(e)}")

