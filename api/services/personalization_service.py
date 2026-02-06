"""Personalization service for adapting content to user background."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .openai_service import get_openai_service


class ExpertiseLevel(str, Enum):
    """User expertise levels."""

    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


@dataclass
class UserBackground:
    """User background information."""

    expertise_level: ExpertiseLevel
    programming_experience: bool
    robotics_experience: bool
    math_comfort: str  # "basic", "intermediate", "advanced"
    learning_goals: list[str]


class PersonalizationService:
    """Service for personalizing content based on user background."""

    def __init__(self):
        self.openai = get_openai_service()

    def determine_content_depth(self, background: UserBackground) -> str:
        """Determine appropriate content depth based on background."""
        if background.expertise_level == ExpertiseLevel.ADVANCED:
            return "advanced"
        elif background.expertise_level == ExpertiseLevel.INTERMEDIATE:
            return "intermediate"
        else:
            return "beginner"

    async def personalize_content(
        self,
        content: str,
        background: UserBackground,
        chapter_slug: str,
    ) -> str:
        """
        Personalize chapter content for user's background.

        Args:
            content: Original chapter content
            background: User's background information
            chapter_slug: Chapter identifier

        Returns:
            Personalized content
        """
        depth = self.determine_content_depth(background)

        # Build personalization prompt
        context_parts = [
            f"User expertise: {background.expertise_level.value}",
            f"Programming experience: {'Yes' if background.programming_experience else 'No'}",
            f"Robotics experience: {'Yes' if background.robotics_experience else 'No'}",
            f"Math comfort level: {background.math_comfort}",
        ]

        if background.learning_goals:
            context_parts.append(
                f"Learning goals: {', '.join(background.learning_goals)}"
            )

        user_context = "\n".join(context_parts)

        system_prompt = f"""You are an educational content adapter. Modify the following textbook content
to better suit the reader's background while maintaining accuracy.

Reader Background:
{user_context}

Adaptation Guidelines for {depth} level:
"""

        if depth == "beginner":
            system_prompt += """
- Add more context and explanations for technical terms
- Include analogies to everyday concepts
- Break down complex concepts into smaller steps
- Add "Why this matters" sections
- Simplify mathematical notation where possible
- Add more code comments and explanations"""

        elif depth == "intermediate":
            system_prompt += """
- Maintain technical accuracy with moderate explanation
- Connect concepts to practical applications
- Include references to related advanced topics
- Balance theory and practice"""

        else:  # advanced
            system_prompt += """
- Use precise technical language
- Include mathematical derivations
- Reference academic papers and advanced resources
- Focus on edge cases and optimizations
- Include performance considerations"""

        system_prompt += """

Important:
- Preserve all code blocks (may add comments for beginners)
- Keep the overall structure intact
- Maintain factual accuracy
- Don't remove critical information, just adjust presentation"""

        messages = [
            {"role": "user", "content": f"Please adapt this content:\n\n{content}"}
        ]

        personalized, _ = await self.openai.chat_completion(
            messages=messages,
            system_prompt=system_prompt,
            temperature=0.5,
            max_tokens=4096,
        )

        return personalized

    def get_content_recommendations(
        self,
        background: UserBackground,
        current_chapter: str,
    ) -> list[dict]:
        """Get recommended next chapters based on user background."""
        recommendations = []

        # Basic recommendations based on expertise
        if background.expertise_level == ExpertiseLevel.BEGINNER:
            if not background.programming_experience:
                recommendations.append(
                    {
                        "chapter": "appendices/b-ros2-installation",
                        "reason": "Start with setting up your development environment",
                    }
                )
            recommendations.append(
                {
                    "chapter": "module-1-ros2/01-intro-physical-ai",
                    "reason": "Foundational concepts in Physical AI",
                }
            )

        elif background.expertise_level == ExpertiseLevel.INTERMEDIATE:
            if background.robotics_experience:
                recommendations.append(
                    {
                        "chapter": "module-3-isaac/14-isaac-overview",
                        "reason": "Leverage your robotics background with NVIDIA Isaac",
                    }
                )
            else:
                recommendations.append(
                    {
                        "chapter": "module-2-simulation/09-gazebo-setup",
                        "reason": "Learn robotics simulation fundamentals",
                    }
                )

        else:  # advanced
            recommendations.append(
                {
                    "chapter": "module-4-vla/25-llm-planning",
                    "reason": "Advanced cognitive planning with LLMs",
                }
            )
            recommendations.append(
                {
                    "chapter": "module-3-isaac/19-reinforcement-learning",
                    "reason": "Deep dive into RL for robot control",
                }
            )

        return recommendations


# Singleton instance
_personalization_service: Optional[PersonalizationService] = None


def get_personalization_service() -> PersonalizationService:
    """Get or create personalization service singleton."""
    global _personalization_service
    if _personalization_service is None:
        _personalization_service = PersonalizationService()
    return _personalization_service
