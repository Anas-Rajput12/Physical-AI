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
  publishableKey?: string; // Make it optional since it might come from environment
}

const AuthProvider: React.FC<AuthProviderProps> = ({ children, publishableKey }) => {
  const resolvedPublishableKey = publishableKey || process.env.NEXT_PUBLIC_CLERK_PUBLISHABLE_KEY || process.env.REACT_APP_CLERK_PUBLISHABLE_KEY;

  // If no publishable key is available, provide a fallback or show error
  if (!resolvedPublishableKey) {
    console.warn("Clerk publishable key is not configured. Authentication will not work.");
    return (
      <div style={{
        padding: '2rem',
        textAlign: 'center',
        backgroundColor: '#f8fafc',
        minHeight: '100vh'
      }}>
        <h2 style={{ color: '#dc2626' }}>Authentication Service Not Configured</h2>
        <p style={{ color: '#6b7280' }}>
          Please contact the administrator to configure the authentication service.
        </p>
        <p style={{ color: '#4b5563', fontSize: '0.9rem', marginTop: '1rem' }}>
          This application requires a Clerk publishable key to enable authentication.
        </p>
      </div>
    );
  }

  // Configure Clerk to disable phone number authentication completely
  const clerkOptions = {
    publishableKey: resolvedPublishableKey,
    signInUrl: '/signin',
    signUpUrl: '/signup',
    signInFallbackRedirectUrl: '/dashboard',
    signUpFallbackRedirectUrl: '/dashboard',
  };

  return (
    <ClerkProvider {...clerkOptions}>
      <AuthProviderComponent>
        {children}
      </AuthProviderComponent>
    </ClerkProvider>
  );
};

export default AuthProvider;