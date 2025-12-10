import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { getToken, removeToken, getMe, UserProfile } from '../lib/auth';

interface AuthContextType {
  isAuthenticated: boolean;
  user: UserProfile | null;
  loading: boolean;
  signOut: () => void;
  refreshUser: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within AuthProvider');
  }
  return context;
};

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const baseUrl = useBaseUrl('/');
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [user, setUser] = useState<UserProfile | null>(null);
  const [loading, setLoading] = useState(true);

  const loadUser = async () => {
    const token = getToken();

    if (!token) {
      setIsAuthenticated(false);
      setUser(null);
      setLoading(false);
      return;
    }

    try {
      const profile = await getMe(token);
      setUser(profile);
      setIsAuthenticated(true);
    } catch (error) {
      console.error('Failed to load user profile:', error);
      // Token invalid/expired, clear it
      removeToken();
      setIsAuthenticated(false);
      setUser(null);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    loadUser();
  }, []);

  const signOut = () => {
    removeToken();
    setIsAuthenticated(false);
    setUser(null);
    // Redirect to signin (use baseUrl for GitHub Pages compatibility)
    window.location.href = `${baseUrl}signin`;
  };

  const refreshUser = async () => {
    await loadUser();
  };

  return (
    <AuthContext.Provider
      value={{
        isAuthenticated,
        user,
        loading,
        signOut,
        refreshUser,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
};

export default AuthProvider;
