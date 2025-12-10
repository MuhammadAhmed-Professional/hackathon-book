/**
 * Authentication client configuration for Physical AI Textbook.
 * Provides helper functions for signup, signin, and token management.
 */

import config from '../config';

export interface SoftwareBackground {
  programming_languages: string[];
  robotics_experience: 'none' | 'beginner' | 'intermediate' | 'advanced';
  ai_ml_level: 'none' | 'basic' | 'intermediate' | 'advanced';
}

export interface HardwareBackground {
  rtx_gpu_access: boolean;
  rtx_gpu_model?: string;
  jetson_kit?: 'none' | 'Nano 4GB' | 'Nano 8GB' | 'Orin Nano 8GB' | 'Orin NX 16GB' | 'AGX Orin 64GB';
  robot_hardware?: string;
}

export interface SignupData {
  email: string;
  password: string;
  software_background: SoftwareBackground;
  hardware_background: HardwareBackground;
}

export interface SigninData {
  email: string;
  password: string;
}

export interface AuthResponse {
  user_id: number;
  email: string;
  token: string;
}

export interface UserProfile {
  user_id: number;
  email: string;
  software_background?: SoftwareBackground;
  hardware_background?: HardwareBackground;
  created_at: string;
  last_login?: string;
}

/**
 * Sign up a new user.
 */
export async function signup(data: SignupData): Promise<AuthResponse> {
  const response = await fetch(`${config.API_BASE_URL}/auth/signup`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    credentials: 'include',
    body: JSON.stringify(data),
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail || 'Signup failed');
  }

  return response.json();
}

/**
 * Sign in an existing user.
 */
export async function signin(data: SigninData): Promise<AuthResponse> {
  const response = await fetch(`${config.API_BASE_URL}/auth/signin`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    credentials: 'include',
    body: JSON.stringify(data),
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail || 'Signin failed');
  }

  return response.json();
}

/**
 * Get current user profile (requires authentication).
 */
export async function getMe(token: string): Promise<UserProfile> {
  const response = await fetch(`${config.API_BASE_URL}/auth/me`, {
    method: 'GET',
    headers: {
      'Authorization': `Bearer ${token}`,
    },
    credentials: 'include',
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail || 'Failed to get user profile');
  }

  return response.json();
}

/**
 * Store authentication token in localStorage.
 */
export function storeToken(token: string): void {
  localStorage.setItem('auth_token', token);
}

/**
 * Get authentication token from localStorage.
 */
export function getToken(): string | null {
  return localStorage.getItem('auth_token');
}

/**
 * Remove authentication token from localStorage.
 */
export function removeToken(): void {
  localStorage.removeItem('auth_token');
}

/**
 * Check if user is authenticated (has valid token).
 */
export function isAuthenticated(): boolean {
  return getToken() !== null;
}
