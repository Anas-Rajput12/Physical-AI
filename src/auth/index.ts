import { createAuth } from "better-auth";

export const auth = createAuth({
  secret: process.env.BETTER_AUTH_SECRET || "fallback-secret-for-development",
  database: {
    provider: "sqlite",
    url: process.env.DATABASE_URL || "./sqlite.db",
  },
  emailAndPassword: {
    enabled: true,
  },
  // Additional configuration for user profiles and onboarding
  user: {
    // Define additional user profile fields for onboarding
    schema: (prev) => ({
      ...prev,
      profile: {
        softwareLevel: "text",
        hardwareExperience: "text",
        programmingLanguages: "text",
        goal: "text",
      },
    }),
  },
});