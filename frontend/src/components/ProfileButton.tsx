import React from 'react';
import { useAuth } from './AuthProvider';
import './ProfileButton.css';

const ProfileButton: React.FC = () => {
  const { isAuthenticated, user, signOut, loading } = useAuth();

  if (loading) {
    return null; // or a loading spinner
  }

  if (!isAuthenticated || !user) {
    return null; // Don't show anything when not authenticated
  }

  return (
    <div className="profile-button-container">
      <div className="profile-dropdown">
        <button className="profile-trigger">
          <span className="profile-email">{user.email}</span>
          <span className="profile-icon">▼</span>
        </button>
        <div className="profile-menu">
          <div className="profile-menu-header">
            <strong>{user.email}</strong>
            {user.software_background && (
              <div className="profile-info">
                <small>
                  {user.software_background.robotics_experience} robotics • {' '}
                  {user.software_background.ai_ml_level} AI/ML
                </small>
              </div>
            )}
          </div>
          <button onClick={signOut} className="profile-signout-btn">
            Sign Out
          </button>
        </div>
      </div>
    </div>
  );
};

export default ProfileButton;
