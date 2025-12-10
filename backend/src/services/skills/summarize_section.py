"""
SummarizeSection skill - Reusable agent skill for summarizing book sections.
"""
import logging
from typing import Dict, Any
from openai import OpenAI

from src.config import config

logger = logging.getLogger(__name__)

# Initialize OpenAI client
openai_client = OpenAI(api_key=config.OPENAI_API_KEY)


# Persona: Educational Content Specialist
PERSONA = """
You are an educational content specialist with expertise in creating clear, concise summaries 
of technical and educational content. Your goal is to help learners quickly understand key 
concepts without losing important information.
"""

# Analytical Questions
ANALYTICAL_QUESTIONS = [
    "What are the key points in this text?",
    "What is the main message or takeaway?",
    "What concepts need to be preserved in the summary?",
    "What can be omitted without losing essential meaning?",
    "How can this be structured for maximum clarity?"
]

# Decision Principles
DECISION_PRINCIPLES = [
    "Conciseness: Summaries should be 20-30% of original length",
    "Accuracy: All facts and concepts must be preserved",
    "Educational Value: Summary should help learners understand, not just reduce text",
    "Structure: Maintain logical flow and hierarchy when possible",
    "Completeness: Cover all major topics, not just one aspect"
]


def summarize_section(text: str, max_length: int = 200) -> Dict[str, Any]:
    """
    Summarize a section of text using Claude Code Subagents pattern.
    
    Args:
        text: Text to summarize
        max_length: Maximum length of summary in words (default: 200)
    
    Returns:
        Dictionary with summary and metadata
    """
    try:
        # Build prompt using Persona + Questions + Principles
        system_prompt = f"""{PERSONA}

When summarizing, consider these questions:
{chr(10).join(f"- {q}" for q in ANALYTICAL_QUESTIONS)}

Follow these principles:
{chr(10).join(f"- {p}" for p in DECISION_PRINCIPLES)}"""

        user_prompt = f"""Please provide a concise summary of the following text (approximately {max_length} words):

{text}

Summary:"""

        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.3,  # Lower temperature for more consistent summaries
            max_tokens=300
        )
        
        summary = response.choices[0].message.content
        
        return {
            "summary": summary,
            "original_length": len(text.split()),
            "summary_length": len(summary.split()),
            "compression_ratio": len(summary.split()) / len(text.split()) if text.split() else 0,
            "model_used": response.model
        }
    
    except Exception as e:
        logger.error(f"Error in summarize_section: {str(e)}")
        raise

