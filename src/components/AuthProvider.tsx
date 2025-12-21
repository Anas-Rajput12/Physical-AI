import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react';
import { getCurrentUser, setCurrentUser, clearCurrentUser, setUserProfile } from '../auth/session';

interface AuthContextType {
  user: any | null;
  loading: boolean;
  signIn: (email: string, password: string) => Promise<void>;
  signOut: () => void;
  signUp: (email: string, password: string, name: string) => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

const AuthProviderComponent: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<any | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Check for existing session on component mount
    const currentUser = getCurrentUser();
    if (currentUser) {
      setUser(currentUser);
    }
    setLoading(false);
  }, []);

  const signIn = async (email: string, password: string) => {
    setLoading(true);
    // Simulate API call
    setTimeout(() => {
      const mockUser = {
        id: 'mock-user-id',
        email: email,
        name: email.split('@')[0],
      };
      setCurrentUser(mockUser);
      setUser(mockUser);
      setLoading(false);
    }, 500);
  };

  const signOut = () => {
    clearCurrentUser();
    setUser(null);
  };

  const signUp = async (email: string, password: string, name: string) => {
    setLoading(true);
    // Simulate API call
    setTimeout(() => {
      const mockUser = {
        id: 'mock-user-id',
        email: email,
        name: name,
      };
      setCurrentUser(mockUser);
      setUser(mockUser);
      setLoading(false);
    }, 500);
  };

  const value = {
    user,
    loading,
    signIn,
    signOut,
    signUp
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  return (
    <AuthProviderComponent>
      {children}
    </AuthProviderComponent>
  );
};

export default AuthProvider;