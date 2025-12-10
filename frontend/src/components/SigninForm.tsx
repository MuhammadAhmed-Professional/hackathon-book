import React, { useState } from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { signin, storeToken } from '../lib/auth';
import './AuthForms.css';

const SigninForm: React.FC = () => {
  const baseUrl = useBaseUrl('/');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    if (!email || !password) {
      setError('Email and password are required');
      return;
    }

    setLoading(true);

    try {
      const response = await signin({ email, password });

      // Store token
      storeToken(response.token);

      // Redirect to textbook (use baseUrl for GitHub Pages compatibility)
      window.location.href = `${baseUrl}docs/intro`;

    } catch (err: any) {
      setError(err.message || 'Sign in failed. Please check your credentials.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-form-container">
      <form onSubmit={handleSubmit} className="auth-form">
        <h2>Sign In to Physical AI Textbook</h2>
        <p className="auth-subtitle">Welcome back! Continue your learning journey.</p>

        {error && <div className="auth-error">{error}</div>}

        <div className="form-group">
          <label htmlFor="email">Email</label>
          <input
            type="email"
            id="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            placeholder="you@example.com"
            required
            autoFocus
          />
        </div>

        <div className="form-group">
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            placeholder="••••••••"
            required
          />
        </div>

        <button type="submit" className="auth-submit-btn" disabled={loading}>
          {loading ? 'Signing In...' : 'Sign In'}
        </button>

        <p className="auth-footer">
          Don't have an account? <a href="/signup">Sign Up</a>
        </p>
      </form>
    </div>
  );
};

export default SigninForm;
