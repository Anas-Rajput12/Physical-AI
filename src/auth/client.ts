import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.NODE_ENV === 'production'
    ? 'https://my-physical-ai-book.vercel.app' // Replace with your production URL
    : 'http://localhost:3000',
  fetchOptions: {
    cache: "no-store",
  },
});