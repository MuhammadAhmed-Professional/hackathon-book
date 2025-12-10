"""
Personalization Service

Provides adaptive content recommendations and filtering based on user profiles.
Supports hardware-specific content, skill-level adaptation, and personalized learning paths.
"""

from typing import Dict, List, Optional, Any
import logging

logger = logging.getLogger(__name__)


class PersonalizationService:
    """Service for personalizing content based on user profiles."""

    # Content tier mappings
    TIER_MAPPING = {
        'none': 'beginner',
        'beginner': 'beginner',
        'basic': 'beginner',
        'intermediate': 'intermediate',
        'advanced': 'advanced'
    }

    # Hardware capability detection
    SUPPORTED_RTX_GPUS = [
        'RTX 2060', 'RTX 2070', 'RTX 2080', 'RTX 3060', 'RTX 3070', 'RTX 3080', 'RTX 3090',
        'RTX 4060', 'RTX 4070', 'RTX 4080', 'RTX 4090', 'RTX 4070 Ti', 'RTX 4090 Ti'
    ]

    @staticmethod
    def get_user_tier(user_profile: Optional[Dict[str, Any]]) -> str:
        """
        Determine user's skill tier from profile.

        Args:
            user_profile: User profile dict with software_background

        Returns:
            'beginner', 'intermediate', or 'advanced'
        """
        if not user_profile or 'software_background' not in user_profile:
            return 'beginner'  # Default to beginner

        software_bg = user_profile['software_background']
        robotics_exp = software_bg.get('robotics_experience', 'none')

        return PersonalizationService.TIER_MAPPING.get(robotics_exp, 'beginner')

    @staticmethod
    def get_hardware_capabilities(user_profile: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Extract hardware capabilities from user profile.

        Args:
            user_profile: User profile dict with hardware_background

        Returns:
            Dict with hardware capability flags
        """
        if not user_profile or 'hardware_background' not in user_profile:
            return {
                'has_rtx_gpu': False,
                'rtx_gpu_model': None,
                'can_run_isaac_sim': False,
                'has_jetson': False,
                'jetson_model': None,
                'can_run_isaac_ros': False,
                'robot_hardware': None
            }

        hardware_bg = user_profile['hardware_background']

        has_rtx = hardware_bg.get('rtx_gpu_access', False)
        rtx_model = hardware_bg.get('rtx_gpu_model', None)

        # Check if RTX GPU is powerful enough for Isaac Sim (RTX 2060+)
        can_run_isaac_sim = False
        if has_rtx and rtx_model:
            can_run_isaac_sim = any(gpu in rtx_model for gpu in PersonalizationService.SUPPORTED_RTX_GPUS)

        jetson_kit = hardware_bg.get('jetson_kit', None)
        has_jetson = jetson_kit is not None and jetson_kit != 'none'

        # Check if Jetson is powerful enough for Isaac ROS (Orin or Xavier)
        can_run_isaac_ros = False
        if has_jetson and jetson_kit:
            can_run_isaac_ros = 'Orin' in jetson_kit or 'Xavier' in jetson_kit

        return {
            'has_rtx_gpu': has_rtx,
            'rtx_gpu_model': rtx_model,
            'can_run_isaac_sim': can_run_isaac_sim,
            'has_jetson': has_jetson,
            'jetson_model': jetson_kit,
            'can_run_isaac_ros': can_run_isaac_ros,
            'robot_hardware': hardware_bg.get('robot_hardware', None)
        }

    @staticmethod
    def get_personalization_context(user_profile: Optional[Dict[str, Any]]) -> str:
        """
        Generate context string for RAG query personalization.

        Args:
            user_profile: User profile dict

        Returns:
            Context string to append to RAG queries
        """
        if not user_profile:
            return ""

        context_parts = []

        # Add skill level context
        tier = PersonalizationService.get_user_tier(user_profile)
        if tier == 'beginner':
            context_parts.append("Explain concepts for beginners with step-by-step instructions.")
        elif tier == 'intermediate':
            context_parts.append("Provide intermediate-level explanations with practical examples.")
        elif tier == 'advanced':
            context_parts.append("Give advanced technical details with research references.")

        # Add hardware context
        hw_caps = PersonalizationService.get_hardware_capabilities(user_profile)

        if hw_caps['can_run_isaac_sim']:
            context_parts.append(f"User has {hw_caps['rtx_gpu_model']}, prioritize NVIDIA Isaac Sim examples.")

        if hw_caps['can_run_isaac_ros']:
            context_parts.append(f"User has {hw_caps['jetson_model']}, include Isaac ROS GEMs examples.")

        if not hw_caps['has_rtx_gpu'] and not hw_caps['has_jetson']:
            context_parts.append("User has standard hardware, prioritize CPU-based alternatives and cloud resources.")

        # Add programming language context
        if 'software_background' in user_profile:
            languages = user_profile['software_background'].get('programming_languages', [])
            if 'Python' in languages and 'C++' not in languages:
                context_parts.append("Focus on Python examples.")
            elif 'C++' in languages:
                context_parts.append("Include C++ examples where appropriate.")

        return " ".join(context_parts)

    @staticmethod
    def filter_content_by_tier(content_items: List[Dict[str, Any]], user_tier: str) -> List[Dict[str, Any]]:
        """
        Filter and rank content items based on user's skill tier.

        Args:
            content_items: List of content items with 'tier' metadata
            user_tier: User's skill level ('beginner', 'intermediate', 'advanced')

        Returns:
            Filtered and ranked content items
        """
        tier_priority = {
            'beginner': {'beginner': 1.0, 'intermediate': 0.3, 'advanced': 0.1},
            'intermediate': {'beginner': 0.5, 'intermediate': 1.0, 'advanced': 0.6},
            'advanced': {'beginner': 0.2, 'intermediate': 0.7, 'advanced': 1.0}
        }

        # Assign relevance scores
        for item in content_items:
            item_tier = item.get('tier', 'intermediate')
            item['tier_relevance'] = tier_priority.get(user_tier, {}).get(item_tier, 0.5)

        # Sort by tier relevance (descending)
        content_items.sort(key=lambda x: x['tier_relevance'], reverse=True)

        return content_items

    @staticmethod
    def get_recommended_chapters(
        user_profile: Optional[Dict[str, Any]],
        completed_chapters: List[str],
        all_chapters: List[Dict[str, str]]
    ) -> List[Dict[str, str]]:
        """
        Recommend next chapters based on user profile and progress.

        Args:
            user_profile: User profile dict
            completed_chapters: List of completed chapter IDs
            all_chapters: List of all available chapters with metadata

        Returns:
            List of recommended chapters (max 5)
        """
        tier = PersonalizationService.get_user_tier(user_profile)
        hw_caps = PersonalizationService.get_hardware_capabilities(user_profile)

        # Filter out completed chapters
        incomplete = [ch for ch in all_chapters if ch['id'] not in completed_chapters]

        # Score each chapter
        for chapter in incomplete:
            score = 0.0

            # Tier match
            chapter_tier = chapter.get('tier', 'intermediate')
            if chapter_tier == tier:
                score += 1.0
            elif abs(ord(chapter_tier[0]) - ord(tier[0])) == 1:  # Adjacent tiers
                score += 0.5

            # Hardware match
            if hw_caps['can_run_isaac_sim'] and 'isaac-sim' in chapter.get('tags', []):
                score += 0.5
            if hw_caps['can_run_isaac_ros'] and 'isaac-ros' in chapter.get('tags', []):
                score += 0.5
            if not hw_caps['has_rtx_gpu'] and 'gazebo' in chapter.get('tags', []):
                score += 0.3  # Prefer Gazebo for users without RTX

            chapter['recommendation_score'] = score

        # Sort by score and return top 5
        incomplete.sort(key=lambda x: x['recommendation_score'], reverse=True)
        return incomplete[:5]

    @staticmethod
    def adapt_code_example(
        code: str,
        language: str,
        user_profile: Optional[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Adapt code example based on user's background.

        Args:
            code: Code snippet
            language: Programming language
            user_profile: User profile dict

        Returns:
            Dict with adapted code and metadata
        """
        tier = PersonalizationService.get_user_tier(user_profile)
        hw_caps = PersonalizationService.get_hardware_capabilities(user_profile)

        # Determine if we should show comments
        add_detailed_comments = tier == 'beginner'

        # Hardware-specific optimizations
        hardware_note = None
        if hw_caps['can_run_isaac_sim'] and 'isaac' in code.lower():
            hardware_note = f"✓ Optimized for your {hw_caps['rtx_gpu_model']}"
        elif hw_caps['can_run_isaac_ros'] and 'isaac_ros' in code.lower():
            hardware_note = f"✓ Ready for your {hw_caps['jetson_model']}"
        elif not hw_caps['has_rtx_gpu'] and 'gpu' in code.lower():
            hardware_note = "ℹ️ This requires a GPU. Try cloud alternatives or CPU version."

        return {
            'code': code,
            'language': language,
            'add_detailed_comments': add_detailed_comments,
            'hardware_note': hardware_note,
            'tier': tier
        }


# Singleton instance
personalization_service = PersonalizationService()
