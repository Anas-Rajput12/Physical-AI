import React, { ReactNode } from 'react';
import { ClerkProvider, useUser, useClerk } from '@clerk/clerk-react';

// Define the context type
interface AuthContextType {
  user: any | null;
  loading: boolean;
  signIn: () => void;
  signOut: () => void;
  signUp: () => void;
}

// Create context
const AuthContext = React.createContext<AuthContextType | undefined>(undefined);

// AuthProvider component that wraps the app
const AuthProviderComponent: React.FC<{ children: ReactNode }> = ({ children }) => {
  const { user, isLoaded, isSignedIn } = useUser();
  const { openSignIn, openSignUp, signOut } = useClerk();

  const signIn = () => {
    openSignIn();
  };

  const signUp = () => {
    openSignUp();
  };

  const handleSignOut = async () => {
    await signOut();
    // Redirect to home page after sign out
    window.location.href = '/my-physical-ai-book/';
  };

  const value = {
    user: isSignedIn && user ? {
      id: user.id,
      email: user.primaryEmailAddress?.emailAddress || null,
      name: user.fullName || user.username || null,
    } : null,
    loading: !isLoaded,
    signIn,
    signOut: handleSignOut,
    signUp
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

// Hook to use the auth context
export const useAuth = () => {
  const context = React.useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

// Export the UserButton for use in headers
export { UserButton } from '@clerk/clerk-react';

// Main AuthProvider that initializes Clerk
interface AuthProviderProps {
  children: ReactNode;
  publishableKey: string;
}

const AuthProvider: React.FC<AuthProviderProps> = ({ children, publishableKey }) => {
  return (
    <ClerkProvider publishableKey={publishableKey}>
      <AuthProviderComponent>
        {children}
      </AuthProviderComponent>
    </ClerkProvider>
  );
};

export default AuthProvider;