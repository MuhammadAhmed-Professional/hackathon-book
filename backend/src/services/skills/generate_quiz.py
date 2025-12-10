"""
GenerateQuizQuestions skill - Reusable agent skill for generating quiz questions.
"""
import logging
from typing import Dict, Any, List
from openai import OpenAI

from src.config import config

logger = logging.getLogger(__name__)

# Initialize OpenAI client
openai_client = OpenAI(api_key=config.OPENAI_API_KEY)


# Persona: Educational Assessment Designer
PERSONA = """
You are an educational assessment designer specializing in creating effective multiple-choice 
quiz questions. Your goal is to test understanding of key concepts while providing fair, 
clear assessment opportunities.
"""

# Analytical Questions
ANALYTICAL_QUESTIONS = [
    "What concepts should be tested from this text?",
    "What difficulty level is appropriate?",
    "What are the key learning objectives?",
    "What common misconceptions should be addressed?",
    "How can questions test deep understanding, not just recall?"
]

# Decision Principles
DECISION_PRINCIPLES = [
    "Clear Questions: Each question should be unambiguous and testable",
    "Plausible Distractors: Wrong answers should be plausible but clearly incorrect",
    "One Correct Answer: Only one answer should be definitively correct",
    "Educational Value: Questions should reinforce learning, not just test",
    "Appropriate Difficulty: Match difficulty to the content level"
]


def generate_quiz_questions(text: str, num_questions: int = 3) -> Dict[str, Any]:
    """
    Generate multiple-choice quiz questions from text using Claude Code Subagents pattern.
    
    Args:
        text: Text to generate questions from
        num_questions: Number of questions to generate (default: 3)
    
    Returns:
        Dictionary with questions and metadata
    """
    try:
        # Build prompt using Persona + Questions + Principles
        system_prompt = f"""{PERSONA}

When creating quiz questions, consider these questions:
{chr(10).join(f"- {q}" for q in ANALYTICAL_QUESTIONS)}

Follow these principles:
{chr(10).join(f"- {p}" for p in DECISION_PRINCIPLES)}"""

        user_prompt = f"""Generate {num_questions} multiple-choice quiz questions based on the following text.
Each question should have 4 options (A, B, C, D) with exactly one correct answer.

Text:
{text}

Format your response as JSON with this structure:
{{
  "questions": [
    {{
      "question": "Question text here",
      "options": ["Option A", "Option B", "Option C", "Option D"],
      "correct_answer": 0,
      "explanation": "Brief explanation of why this is correct"
    }}
  ]
}}"""

        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.7,
            max_tokens=1000,
            response_format={"type": "json_object"}
        )
        
        import json
        result = json.loads(response.choices[0].message.content)
        
        return {
            "questions": result.get("questions", []),
            "num_questions": len(result.get("questions", [])),
            "model_used": response.model
        }
    
    except Exception as e:
        logger.error(f"Error in generate_quiz_questions: {str(e)}")
        raise

