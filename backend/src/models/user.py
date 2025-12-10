"""
User models for Physical AI Textbook authentication and profiling.
Feature: 002-physical-ai-book
"""

from pydantic import BaseModel, EmailStr
from typing import Optional, List
from datetime import datetime


class SoftwareBackground(BaseModel):
    """User's software development and AI/ML background."""
    programming_languages: List[str]
    robotics_experience: str  # "none" | "beginner" | "intermediate" | "advanced"
    ai_ml_level: str  # "none" | "basic" | "intermediate" | "advanced"
    prior_courses: Optional[List[str]] = []


class HardwareBackground(BaseModel):
    """User's robotics hardware access and experience."""
    rtx_gpu_access: bool
    rtx_gpu_model: Optional[str] = None
    jetson_kit: str  # "none" | "Orin Nano 8GB" | "Orin NX 16GB" | other
    robot_hardware: str  # "none" | "quadruped" | "humanoid" | "robotic_arm" | other


class User(BaseModel):
    """Complete user profile (returned from database)."""
    id: int
    email: EmailStr
    software_background: Optional[SoftwareBackground] = None
    hardware_background: Optional[HardwareBackground] = None
    created_at: datetime
    last_login: Optional[datetime] = None


class UserCreate(BaseModel):
    """User signup request with background questionnaire."""
    email: EmailStr
    password: Optional[str] = None  # Optional - password hash is passed separately to create_user
    software_background: SoftwareBackground
    hardware_background: HardwareBackground


class UserLogin(BaseModel):
    """User signin request."""
    email: EmailStr
    password: str
