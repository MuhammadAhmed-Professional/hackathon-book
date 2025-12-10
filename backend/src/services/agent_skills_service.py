"""
Agent Skills service for executing specialized AI skills.
Feature: 002-physical-ai-book (Phase 9: Agent Skills)
"""
import logging
import os
from typing import Dict, Any, Optional
from openai import OpenAI
from pathlib import Path

from src.config import config

logger = logging.getLogger(__name__)

# Initialize OpenAI client
openai_client = OpenAI(api_key=config.OPENAI_API_KEY)

# Path to skills directory
SKILLS_DIR = Path(__file__).parent.parent.parent.parent / ".specify" / "skills"


def load_skill_prompt(skill_name: str) -> str:
    """
    Load a skill document from the .specify/skills/ directory.

    Args:
        skill_name: Name of the skill file (without .md extension)

    Returns:
        Full skill document content as string

    Raises:
        FileNotFoundError: If skill file doesn't exist
    """
    skill_path = SKILLS_DIR / f"{skill_name}.md"

    if not skill_path.exists():
        raise FileNotFoundError(f"Skill file not found: {skill_path}")

    with open(skill_path, 'r', encoding='utf-8') as f:
        skill_content = f.read()

    logger.info(f"Loaded skill document: {skill_name}")
    return skill_content


def execute_skill(skill_name: str, user_input: str, context: Optional[str] = None) -> Dict[str, Any]:
    """
    Execute an agent skill using OpenAI with the skill's P+Q+P documentation.

    Args:
        skill_name: Name of the skill to execute (e.g., "summarize-section")
        user_input: User's input/request for the skill
        context: Optional context (e.g., chapter content, term context)

    Returns:
        Dictionary with:
            - output: Generated output from the skill
            - model_used: OpenAI model used
            - skill_name: Name of executed skill

    Raises:
        FileNotFoundError: If skill doesn't exist
        Exception: If OpenAI API call fails
    """
    try:
        # Load the skill document
        skill_prompt = load_skill_prompt(skill_name)

        # Build system prompt with skill documentation
        system_prompt = f"""You are an AI assistant executing a specialized skill.

{skill_prompt}

IMPORTANT INSTRUCTIONS:
1. Follow the Persona, Pre-Execution Questions, and Execution Principles EXACTLY as documented above.
2. Generate output in the exact format specified in the skill documentation.
3. Maintain high quality and accuracy as per the quality checklist.
4. If the skill requires context (e.g., chapter content), it will be provided in the user message.
"""

        # Build user prompt
        if context:
            user_prompt = f"""Context/Content:
{context}

User Request:
{user_input}

Please execute the skill using the context provided above."""
        else:
            user_prompt = f"""User Request:
{user_input}

Please execute the skill."""

        # Call OpenAI API with skill prompt
        response = openai_client.chat.completions.create(
            model="gpt-4o-mini",  # Use GPT-4 for high-quality skill execution
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.7,
            max_tokens=3000  # Higher token limit for comprehensive outputs
        )

        output_text = response.choices[0].message.content
        model_used = response.model

        logger.info(f"Executed skill '{skill_name}' using {model_used}")

        return {
            "output": output_text,
            "model_used": model_used,
            "skill_name": skill_name
        }

    except FileNotFoundError as e:
        logger.error(f"Skill not found: {skill_name}")
        raise
    except Exception as e:
        logger.error(f"Error executing skill '{skill_name}': {str(e)}")
        raise


def summarize_section(section_content: str, tier: Optional[str] = None) -> Dict[str, Any]:
    """
    Execute the SummarizeSection skill to generate multi-tier summaries.

    Args:
        section_content: Full content of the section/chapter to summarize
        tier: Optional tier to generate ('tldr', 'standard', 'detailed', or None for all)

    Returns:
        Dictionary with summary output
    """
    user_input = f"Generate a summary for this textbook section."
    if tier:
        user_input += f" Focus on the {tier} tier."

    return execute_skill("summarize-section", user_input, context=section_content)


def generate_quiz_questions(chapter_content: str, num_questions: int = 10, difficulty: Optional[str] = None) -> Dict[str, Any]:
    """
    Execute the GenerateQuizQuestions skill to create assessment questions.

    Args:
        chapter_content: Full content of the chapter for quiz generation
        num_questions: Number of questions to generate (default: 10)
        difficulty: Optional difficulty filter ('easy', 'medium', 'hard', or None for mixed)

    Returns:
        Dictionary with quiz questions in JSON format
    """
    user_input = f"Generate {num_questions} quiz questions for this chapter."
    if difficulty:
        user_input += f" Focus on {difficulty} difficulty."

    return execute_skill("generate-quiz-questions", user_input, context=chapter_content)


def explain_term(term: str, context: Optional[str] = None, tier: Optional[str] = None) -> Dict[str, Any]:
    """
    Execute the ExplainTerm skill to generate multi-level explanations.

    Args:
        term: Technical term to explain (e.g., "DDS", "quaternion", "rclpy.spin()")
        context: Optional context where the term appeared (paragraph, code snippet)
        tier: Optional tier to focus on ('eli5', 'standard', 'technical', or None for all)

    Returns:
        Dictionary with term explanation
    """
    user_input = f"Explain the term: '{term}'"
    if tier:
        user_input += f" Focus on the {tier} explanation tier."

    term_context = context if context else f"This term appeared in the Physical AI textbook."

    return execute_skill("explain-term", user_input, context=term_context)


# Skill registry mapping skill IDs to execution functions
SKILL_REGISTRY = {
    "summarize": summarize_section,
    "quiz": generate_quiz_questions,
    "explain": explain_term
}


def get_available_skills() -> Dict[str, str]:
    """
    Get list of available agent skills with descriptions.

    Returns:
        Dictionary mapping skill IDs to descriptions
    """
    return {
        "summarize": "Generate multi-tier summaries (TL;DR, Standard, Detailed) of textbook sections",
        "quiz": "Create adaptive quiz questions with difficulty tiers and rich feedback",
        "explain": "Provide multi-level explanations (ELI5, Standard, Technical) of technical terms"
    }
