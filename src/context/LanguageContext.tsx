import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

interface LanguageContextType {
  currentLanguage: string;
  languages: { code: string; name: string }[];
  changeLanguage: (lang: string) => void;
}

const LanguageContext = createContext<LanguageContextType | undefined>(undefined);

const languages = [
  { code: 'en', name: 'English' },
  { code: 'ur', name: 'Urdu' },
  { code: 'sd', name: 'Sindhi' },
];

export const LanguageProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [currentLanguage, setCurrentLanguage] = useState<string>(() => {
    // Check localStorage first, then default to 'en'
    if (typeof window !== 'undefined') {
      return localStorage.getItem('preferredLanguage') || 'en';
    }
    return 'en';
  });

  useEffect(() => {
    // Save to localStorage whenever language changes
    localStorage.setItem('preferredLanguage', currentLanguage);
  }, [currentLanguage]);

  useEffect(() => {
    // Listen for language change events from the navbar
    const handleLanguageChange = (e: Event) => {
      const customEvent = e as CustomEvent;
      const newLanguage = customEvent.detail.language;
      if (languages.some(l => l.code === newLanguage)) {
        setCurrentLanguage(newLanguage);
      }
    };

    window.addEventListener('languageChanged', handleLanguageChange);

    return () => {
      window.removeEventListener('languageChanged', handleLanguageChange);
    };
  }, []);

  const changeLanguage = (lang: string) => {
    if (languages.some(l => l.code === lang)) {
      setCurrentLanguage(lang);
      // Also update localStorage and dispatch event for navbar sync
      localStorage.setItem('preferredLanguage', lang);
      if (typeof window !== 'undefined') {
        window.dispatchEvent(new CustomEvent('languageChanged', {
          detail: { language: lang }
        }));
      }
    }
  };

  const value = {
    currentLanguage,
    languages,
    changeLanguage,
  };

  return (
    <LanguageContext.Provider value={value}>
      {children}
    </LanguageContext.Provider>
  );
};

export const useLanguage = () => {
  const context = useContext(LanguageContext);
  if (context === undefined) {
    throw new Error('useLanguage must be used within a LanguageProvider');
  }
  return context;
};