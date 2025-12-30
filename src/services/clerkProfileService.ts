interface UserProfile {
  softwareLevel?: string;
  hardwareExperience?: string;
  programmingLanguages?: string;
  goal?: string;
  createdAt?: string;
}

// Service to handle user profile updates in Clerk
// This function should be called from within a component where the user context is available
export const updateUserProfile = async (user: any, profileData: UserProfile) => {
  try {
    if (!user) {
      throw new Error('User not authenticated');
    }

    // Update user with metadata
    await user.update({
      unsafeMetadata: {
        ...user.unsafeMetadata,
        ...profileData
      }
    });

    console.log('User profile updated successfully');
    return true;
  } catch (error) {
    console.error('Error updating user profile:', error);
    throw error;
  }
};

// Service to get user profile from Clerk metadata
export const getUserProfile = (user: any) => {
  if (!user) {
    return null;
  }

  return {
    id: user.id,
    email: user.primaryEmailAddress?.emailAddress,
    name: user.fullName || user.username,
    ...user.unsafeMetadata as UserProfile
  };
};