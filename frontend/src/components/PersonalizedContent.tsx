import React, { useEffect, useState } from 'react';
import { useAuth } from './AuthProvider';
import config from '../config';
import './PersonalizedContent.css';

interface PersonalizationData {
  user_tier: string;
  hardware_capabilities: {
    has_rtx_gpu: boolean;
    rtx_gpu_model: string | null;
    can_run_isaac_sim: boolean;
    has_jetson: boolean;
    jetson_model: string | null;
    can_run_isaac_ros: boolean;
    robot_hardware: string | null;
  };
  personalization_context: string;
  recommended_content_tier: string;
}

interface ContentVariant {
  tier: 'beginner' | 'intermediate' | 'advanced';
  content: React.ReactNode;
}

interface HardwareContent {
  rtxGPU?: React.ReactNode;
  jetson?: React.ReactNode;
  default: React.ReactNode;
}

interface PersonalizedContentProps {
  variants: ContentVariant[];
  hardware?: HardwareContent;
  className?: string;
}

const PersonalizedContent: React.FC<PersonalizedContentProps> = ({ variants, hardware, className }) => {
  const { isAuthenticated, user } = useAuth();
  const [personalization, setPersonalization] = useState<PersonalizationData | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const loadPersonalization = async () => {
      if (isAuthenticated && user) {
        try {
          const token = localStorage.getItem('auth_token');
          const response = await fetch(`${config.API_BASE_URL}/personalization`, {
            headers: {
              'Authorization': `Bearer ${token}`
            }
          });

          if (response.ok) {
            const data = await response.json();
            setPersonalization(data);
          }
        } catch (error) {
          console.error('Failed to load personalization:', error);
        }
      }
      setLoading(false);
    };

    loadPersonalization();
  }, [isAuthenticated, user]);

  if (loading) {
    return <div className="personalized-content-loading">Loading personalized content...</div>;
  }

  // Determine tier (default to beginner if not logged in)
  const tier = personalization?.user_tier || 'beginner';

  // Select appropriate content variant
  const contentVariant = variants.find(v => v.tier === tier) || variants[0];

  // Determine hardware variant (if applicable)
  let hardwareContent = hardware?.default;
  if (hardware && personalization) {
    if (personalization.hardware_capabilities.can_run_isaac_sim && hardware.rtxGPU) {
      hardwareContent = hardware.rtxGPU;
    } else if (personalization.hardware_capabilities.can_run_isaac_ros && hardware.jetson) {
      hardwareContent = hardware.jetson;
    }
  }

  return (
    <div className={`personalized-content ${className || ''}`} data-tier={tier}>
      {/* Tier Badge */}
      {personalization && (
        <div className="content-tier-badge" data-tier={tier}>
          <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor">
            <path d="M12 2L15.09 8.26L22 9.27L17 14.14L18.18 21.02L12 17.77L5.82 21.02L7 14.14L2 9.27L8.91 8.26L12 2Z" />
          </svg>
          <span>{tier.charAt(0).toUpperCase() + tier.slice(1)} Level</span>
        </div>
      )}

      {/* Main Content */}
      {contentVariant.content}

      {/* Hardware-Specific Content */}
      {hardwareContent && (
        <div className="hardware-specific-section">
          <div className="hardware-section-header">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <rect x="2" y="3" width="20" height="14" rx="2" ry="2"></rect>
              <line x1="8" y1="21" x2="16" y2="21"></line>
              <line x1="12" y1="17" x2="12" y2="21"></line>
            </svg>
            <h3>Optimized for Your Hardware</h3>
          </div>
          {personalization?.hardware_capabilities.can_run_isaac_sim && (
            <div className="hardware-badge rtx">
              ✓ {personalization.hardware_capabilities.rtx_gpu_model}
            </div>
          )}
          {personalization?.hardware_capabilities.can_run_isaac_ros && (
            <div className="hardware-badge jetson">
              ✓ {personalization.hardware_capabilities.jetson_model}
            </div>
          )}
          <div className="hardware-content">
            {hardwareContent}
          </div>
        </div>
      )}

      {/* Upgrade Suggestion for Unauthenticated Users */}
      {!isAuthenticated && (
        <div className="upgrade-suggestion">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M12 15v2m0-6v2m0-6v.01M19 21H5a2 2 0 01-2-2V5a2 2 0 012-2h14a2 2 0 012 2v14a2 2 0 01-2 2z"></path>
          </svg>
          <div>
            <strong>Want personalized content?</strong>
            <p><a href="/signup">Sign up</a> to get content adapted to your hardware and skill level!</p>
          </div>
        </div>
      )}
    </div>
  );
};

export default PersonalizedContent;
