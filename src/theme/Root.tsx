import React from 'react';
import type { Props } from '@theme/Root';
import { useThemeConfig } from '@docusaurus/theme-common';
import RagChatbot from '../components/RagChatbot';
import AuthProvider from '../components/ClerkAuthProvider';
import { LanguageProvider } from '../context/LanguageContext';

interface ThemeConfig {
  clerkPublishableKey?: string;
}

export default function Root({ children }: Props) {
  const themeConfig = useThemeConfig() as any as ThemeConfig;
  // First try to get the key from theme config, then fall back to environment variable
  const clerkPublishableKey = themeConfig.clerkPublishableKey || process.env.REACT_APP_CLERK_PUBLISHABLE_KEY || process.env.NEXT_PUBLIC_CLERK_PUBLISHABLE_KEY || '';

  if (!clerkPublishableKey) {
    console.warn('Clerk publishable key is not set. Please configure it properly in your docusaurus.config.js themeConfig.');
    // If no key is provided, render without Clerk authentication
    return React.createElement(
      LanguageProvider,
      null,
      React.createElement(React.Fragment, null, children, React.createElement(RagChatbot, null))
    );
  }

  return React.createElement(
    LanguageProvider,
    null,
    React.createElement(
      AuthProvider,
      { publishableKey: clerkPublishableKey },
      React.createElement(React.Fragment, null, children, React.createElement(RagChatbot, null))
    )
  );
}
