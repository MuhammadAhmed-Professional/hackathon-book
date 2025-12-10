"""
Personalization API endpoint for content recommendations and profile-based adaptations.
"""
from fastapi import APIRouter, HTTPException, Depends, Header
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import logging

from src.services.auth_service import verify_token
from src.services.db_service import get_user_by_id
from src.services.personalization_service import personalization_service

logger = logging.getLogger(__name__)

router = APIRouter()


class PersonalizationResponse(BaseModel):
    """Response model for personalization data."""
    user_tier: str
    hardware_capabilities: Dict[str, Any]
    personalization_context: str
    recommended_content_tier: str


class RecommendedChapter(BaseModel):
    """Model for recommended chapter."""
    id: str
    title: str
    module: str
    tier: str
    tags: List[str]
    recommendation_score: float
    reason: str


class RecommendationsResponse(BaseModel):
    """Response model for chapter recommendations."""
    recommended_chapters: List[RecommendedChapter]
    completed_count: int
    total_count: int


@router.get("/personalization", response_model=PersonalizationResponse)
async def get_personalization_data(authorization: Optional[str] = Header(None)):
    """
    Get personalization data for the authenticated user.

    Args:
        authorization: JWT token in Authorization header

    Returns:
        PersonalizationResponse with user tier, capabilities, and context
    """
    try:
        user_profile = None

        # Get user profile if authenticated
        if authorization and authorization.startswith("Bearer "):
            token = authorization[7:]
            try:
                payload = verify_token(token)
                user_id = payload["user_id"]

                user_data = get_user_by_id(user_id)
                if user_data:
                    user_profile = {
                        'software_background': user_data.get('software_background', {}),
                        'hardware_background': user_data.get('hardware_background', {})
                    }
            except Exception as e:
                logger.warning(f"Could not load user profile: {str(e)}")

        # Get personalization data
        user_tier = personalization_service.get_user_tier(user_profile)
        hw_caps = personalization_service.get_hardware_capabilities(user_profile)
        context = personalization_service.get_personalization_context(user_profile)

        return PersonalizationResponse(
            user_tier=user_tier,
            hardware_capabilities=hw_caps,
            personalization_context=context,
            recommended_content_tier=user_tier
        )

    except Exception as e:
        logger.error(f"Error getting personalization data: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error retrieving personalization data: {str(e)}"
        )


@router.get("/personalization/recommendations", response_model=RecommendationsResponse)
async def get_recommendations(authorization: Optional[str] = Header(None)):
    """
    Get personalized chapter recommendations.

    Args:
        authorization: JWT token in Authorization header

    Returns:
        RecommendationsResponse with recommended chapters
    """
    try:
        user_profile = None
        completed_chapters = []

        # Get user profile and progress if authenticated
        if authorization and authorization.startswith("Bearer "):
            token = authorization[7:]
            try:
                payload = verify_token(token)
                user_id = payload["user_id"]

                user_data = get_user_by_id(user_id)
                if user_data:
                    user_profile = {
                        'software_background': user_data.get('software_background', {}),
                        'hardware_background': user_data.get('hardware_background', {})
                    }
                    # TODO: Get completed chapters from database
                    completed_chapters = []  # Placeholder
            except Exception as e:
                logger.warning(f"Could not load user data: {str(e)}")

        # Define available chapters (this should come from a chapters service/database)
        all_chapters = [
            {
                'id': 'module1/ros2-architecture',
                'title': 'ROS 2 Architecture',
                'module': 'Module 1',
                'tier': 'beginner',
                'tags': ['ros2', 'fundamentals']
            },
            {
                'id': 'module1/nodes-topics-services',
                'title': 'Nodes, Topics, and Services',
                'module': 'Module 1',
                'tier': 'beginner',
                'tags': ['ros2', 'communication']
            },
            {
                'id': 'module1/python-integration',
                'title': 'Python Integration with ROS 2',
                'module': 'Module 1',
                'tier': 'intermediate',
                'tags': ['ros2', 'python', 'rclpy']
            },
            {
                'id': 'module1/urdf-for-humanoids',
                'title': 'URDF for Humanoid Robots',
                'module': 'Module 1',
                'tier': 'intermediate',
                'tags': ['urdf', 'modeling']
            },
            {
                'id': 'module2/gazebo-simulation',
                'title': 'Gazebo Simulation Fundamentals',
                'module': 'Module 2',
                'tier': 'intermediate',
                'tags': ['gazebo', 'simulation']
            },
            {
                'id': 'module3/isaac-sim',
                'title': 'NVIDIA Isaac Sim',
                'module': 'Module 3',
                'tier': 'advanced',
                'tags': ['isaac-sim', 'gpu', 'simulation']
            },
            {
                'id': 'module3/isaac-ros',
                'title': 'Isaac ROS: GPU-Accelerated Perception',
                'module': 'Module 3',
                'tier': 'advanced',
                'tags': ['isaac-ros', 'gpu', 'perception']
            }
        ]

        # Get recommendations
        recommendations = personalization_service.get_recommended_chapters(
            user_profile,
            completed_chapters,
            all_chapters
        )

        # Add reasoning for recommendations
        hw_caps = personalization_service.get_hardware_capabilities(user_profile)
        user_tier = personalization_service.get_user_tier(user_profile)

        recommended_with_reasons = []
        for rec in recommendations:
            reason_parts = []

            # Tier match
            if rec.get('tier') == user_tier:
                reason_parts.append(f"Matches your {user_tier} level")

            # Hardware match
            if hw_caps['can_run_isaac_sim'] and 'isaac-sim' in rec.get('tags', []):
                reason_parts.append(f"Works with your {hw_caps['rtx_gpu_model']}")
            if hw_caps['can_run_isaac_ros'] and 'isaac-ros' in rec.get('tags', []):
                reason_parts.append(f"Optimized for your {hw_caps['jetson_model']}")

            # Default reason
            if not reason_parts:
                reason_parts.append("Recommended next step in your learning path")

            recommended_with_reasons.append(RecommendedChapter(
                id=rec['id'],
                title=rec['title'],
                module=rec['module'],
                tier=rec['tier'],
                tags=rec['tags'],
                recommendation_score=rec.get('recommendation_score', 0.0),
                reason=", ".join(reason_parts)
            ))

        return RecommendationsResponse(
            recommended_chapters=recommended_with_reasons,
            completed_count=len(completed_chapters),
            total_count=len(all_chapters)
        )

    except Exception as e:
        logger.error(f"Error getting recommendations: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error retrieving recommendations: {str(e)}"
        )
