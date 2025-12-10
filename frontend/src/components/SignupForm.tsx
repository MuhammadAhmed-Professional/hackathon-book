import React, { useState } from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { signup, storeToken, SignupData } from '../lib/auth';
import './AuthForms.css';

const SignupForm: React.FC = () => {
  const baseUrl = useBaseUrl('/');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');

  // Software background
  const [programmingLanguages, setProgrammingLanguages] = useState<string[]>([]);
  const [roboticsExperience, setRoboticsExperience] = useState<'none' | 'beginner' | 'intermediate' | 'advanced'>('none');
  const [aiMlLevel, setAiMlLevel] = useState<'none' | 'basic' | 'intermediate' | 'advanced'>('none');

  // Hardware background
  const [rtxGpuAccess, setRtxGpuAccess] = useState(false);
  const [rtxGpuModel, setRtxGpuModel] = useState('');
  const [jetsonKit, setJetsonKit] = useState<'none' | 'Nano 4GB' | 'Nano 8GB' | 'Orin Nano 8GB' | 'Orin NX 16GB' | 'AGX Orin 64GB'>('none');
  const [robotHardware, setRobotHardware] = useState('none');

  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const availableLanguages = [
    'Python', 'C++', 'C', 'Java', 'JavaScript', 'TypeScript',
    'MATLAB', 'Julia', 'Rust', 'Go', 'Other'
  ];

  const handleLanguageToggle = (lang: string) => {
    setProgrammingLanguages(prev =>
      prev.includes(lang)
        ? prev.filter(l => l !== lang)
        : [...prev, lang]
    );
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    // Validation
    if (!email || !password) {
      setError('Email and password are required');
      return;
    }

    if (password.length < 8) {
      setError('Password must be at least 8 characters');
      return;
    }

    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (programmingLanguages.length === 0) {
      setError('Please select at least one programming language');
      return;
    }

    setLoading(true);

    try {
      const signupData: SignupData = {
        email,
        password,
        software_background: {
          programming_languages: programmingLanguages,
          robotics_experience: roboticsExperience,
          ai_ml_level: aiMlLevel,
        },
        hardware_background: {
          rtx_gpu_access: rtxGpuAccess,
          rtx_gpu_model: rtxGpuAccess ? rtxGpuModel : undefined,
          jetson_kit: jetsonKit,
          robot_hardware: robotHardware,
        },
      };

      const response = await signup(signupData);

      // Store token
      storeToken(response.token);

      // Redirect to textbook homepage
      // Redirect to textbook (use baseUrl for GitHub Pages compatibility)
      window.location.href = `${baseUrl}docs/intro`;

    } catch (err: any) {
      setError(err.message || 'Signup failed. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-form-container">
      <form onSubmit={handleSubmit} className="auth-form">
        <h2>Sign Up for Physical AI Textbook</h2>
        <p className="auth-subtitle">Create your account and tell us about your background</p>

        {error && <div className="auth-error">{error}</div>}

        {/* Account Information */}
        <div className="auth-section">
          <h3>Account Information</h3>

          <div className="form-group">
            <label htmlFor="email">Email *</label>
            <input
              type="email"
              id="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              placeholder="you@example.com"
              required
            />
          </div>

          <div className="form-group">
            <label htmlFor="password">Password * (min 8 characters)</label>
            <input
              type="password"
              id="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="••••••••"
              required
            />
          </div>

          <div className="form-group">
            <label htmlFor="confirmPassword">Confirm Password *</label>
            <input
              type="password"
              id="confirmPassword"
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              placeholder="••••••••"
              required
            />
          </div>
        </div>

        {/* Software Background */}
        <div className="auth-section">
          <h3>Software Background</h3>

          <div className="form-group">
            <label>Programming Languages * (select all that apply)</label>
            <div className="checkbox-group">
              {availableLanguages.map((lang) => (
                <label key={lang} className="checkbox-label">
                  <input
                    type="checkbox"
                    checked={programmingLanguages.includes(lang)}
                    onChange={() => handleLanguageToggle(lang)}
                  />
                  {lang}
                </label>
              ))}
            </div>
          </div>

          <div className="form-group">
            <label htmlFor="roboticsExperience">Robotics Experience *</label>
            <select
              id="roboticsExperience"
              value={roboticsExperience}
              onChange={(e) => setRoboticsExperience(e.target.value as any)}
            >
              <option value="none">No experience</option>
              <option value="beginner">Beginner (hobbyist, learning basics)</option>
              <option value="intermediate">Intermediate (built robots, ROS knowledge)</option>
              <option value="advanced">Advanced (professional/research experience)</option>
            </select>
          </div>

          <div className="form-group">
            <label htmlFor="aiMlLevel">AI/ML Knowledge Level *</label>
            <select
              id="aiMlLevel"
              value={aiMlLevel}
              onChange={(e) => setAiMlLevel(e.target.value as any)}
            >
              <option value="none">No experience</option>
              <option value="basic">Basic (understand concepts, minimal coding)</option>
              <option value="intermediate">Intermediate (trained models, familiar with frameworks)</option>
              <option value="advanced">Advanced (research/production ML experience)</option>
            </select>
          </div>
        </div>

        {/* Hardware Background */}
        <div className="auth-section">
          <h3>Hardware Background</h3>

          <div className="form-group">
            <label className="checkbox-label">
              <input
                type="checkbox"
                checked={rtxGpuAccess}
                onChange={(e) => setRtxGpuAccess(e.target.checked)}
              />
              I have access to an NVIDIA RTX GPU
            </label>
          </div>

          {rtxGpuAccess && (
            <div className="form-group">
              <label htmlFor="rtxGpuModel">RTX GPU Model</label>
              <input
                type="text"
                id="rtxGpuModel"
                value={rtxGpuModel}
                onChange={(e) => setRtxGpuModel(e.target.value)}
                placeholder="e.g., RTX 4070 Ti, RTX 3060"
              />
            </div>
          )}

          <div className="form-group">
            <label htmlFor="jetsonKit">NVIDIA Jetson Kit</label>
            <select
              id="jetsonKit"
              value={jetsonKit}
              onChange={(e) => setJetsonKit(e.target.value as any)}
            >
              <option value="none">No Jetson kit</option>
              <option value="Nano 4GB">Jetson Nano 4GB</option>
              <option value="Nano 8GB">Jetson Nano 8GB</option>
              <option value="Orin Nano 8GB">Orin Nano 8GB</option>
              <option value="Orin NX 16GB">Orin NX 16GB</option>
              <option value="AGX Orin 64GB">AGX Orin 64GB</option>
            </select>
          </div>

          <div className="form-group">
            <label htmlFor="robotHardware">Robot Hardware Access</label>
            <select
              id="robotHardware"
              value={robotHardware}
              onChange={(e) => setRobotHardware(e.target.value)}
            >
              <option value="none">No robot hardware</option>
              <option value="arm">Robotic arm (UR5, Franka, etc.)</option>
              <option value="mobile">Mobile robot (TurtleBot, etc.)</option>
              <option value="humanoid">Humanoid (Unitree G1, etc.)</option>
              <option value="drone">Drone/UAV</option>
              <option value="other">Other</option>
            </select>
          </div>
        </div>

        <button type="submit" className="auth-submit-btn" disabled={loading}>
          {loading ? 'Creating Account...' : 'Sign Up'}
        </button>

        <p className="auth-footer">
          Already have an account? <a href="/signin">Sign In</a>
        </p>
      </form>
    </div>
  );
};

export default SignupForm;
