// This file exposes the Clerk publishable key to the client
// It will be bundled with the client-side code

// For development, you can temporarily set the key here
// But for security, make sure to use environment variables in production
const CLERK_PUBLISHABLE_KEY = process.env.REACT_APP_CLERK_PUBLISHABLE_KEY || '';

// Expose the key globally so it can be accessed in the Root component
if (typeof window !== 'undefined') {
  window.__CLERK_PUBLISHABLE_KEY__ = CLERK_PUBLISHABLE_KEY;
}

export { CLERK_PUBLISHABLE_KEY };