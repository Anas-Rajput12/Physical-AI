// Function to get current user
export const getCurrentUser = () => {
  if (typeof window !== 'undefined') {
    const storedUser = localStorage.getItem('currentUser');
    return storedUser ? JSON.parse(storedUser) : null;
  }
  return null;
};

// Function to set current user
export const setCurrentUser = (user: any) => {
  if (typeof window !== 'undefined') {
    localStorage.setItem('currentUser', JSON.stringify(user));
  }
};

// Function to clear current user
export const clearCurrentUser = () => {
  if (typeof window !== 'undefined') {
    localStorage.removeItem('currentUser');
  }
};

// Function to get user profile
export const getUserProfile = () => {
  if (typeof window !== 'undefined') {
    const storedProfile = localStorage.getItem('userProfile');
    return storedProfile ? JSON.parse(storedProfile) : null;
  }
  return null;
};

// Function to set user profile
export const setUserProfile = (profile: any) => {
  if (typeof window !== 'undefined') {
    localStorage.setItem('userProfile', JSON.stringify(profile));
  }
};