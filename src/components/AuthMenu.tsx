import React, { useState, useEffect } from 'react';

// A component that will be loaded after ClerkProvider is set up
// This will be used separately from the Navbar theme component
const AuthMenu = () => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const [clerkLoaded, setClerkLoaded] = useState(false);

  useEffect(() => {
    // Dynamically load Clerk components after the page loads
    const loadClerk = async () => {
      try {
        // Check if Clerk is available
        if (typeof window !== 'undefined' && window.Clerk) {
          // Clerk is loaded, we can now access user state
          // For now, we'll use a simple approach with direct API calls
          const clerk = window.Clerk;
          if (clerk && clerk.session) {
            // Get current user info
            const currentUser = clerk.session.user;
            if (currentUser) {
              setUser({
                name: currentUser.firstName || currentUser.username || currentUser.emailAddresses?.[0]?.emailAddress || 'User'
              });
            }
          }
        }
        setClerkLoaded(true);
        setLoading(false);
      } catch (error) {
        console.error('Error loading Clerk:', error);
        setLoading(false);
      }
    };

    loadClerk();
  }, []);

  const handleSignIn = () => {
    if (typeof window !== 'undefined' && window.Clerk) {
      window.Clerk.openSignIn();
    }
  };

  const handleSignOut = () => {
    if (typeof window !== 'undefined' && window.Clerk) {
      window.Clerk.signOut();
      window.location.reload();
    }
  };

  if (loading) {
    return <div>Loading...</div>;
  }

  return (
    <div className="navbar__item navbar__dropdown">
      <a
        className="navbar__link"
        href="#"
        onClick={(e) => {
          e.preventDefault();
          if (user) {
            handleSignOut();
          } else {
            handleSignIn();
          }
        }}
      >
        {user ? `Hi, ${user.name}` : 'Sign In'}
      </a>
    </div>
  );
};

export default AuthMenu;