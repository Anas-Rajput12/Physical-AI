import React from 'react';
import Navbar from '@theme-original/Navbar';

// Import the UserButton from our auth provider
const { UserButton } = require('../../components/ClerkAuthProvider');

export default function NavbarWrapper(props) {
  return (
    <>
      <Navbar {...props} />
      {/* Add a placeholder for authentication status that will be populated by Clerk */}
      <div id="clerk-auth-container" style={{ position: 'absolute', right: '20px', top: '50%', transform: 'translateY(-50%)' }}></div>
    </>
  );
}