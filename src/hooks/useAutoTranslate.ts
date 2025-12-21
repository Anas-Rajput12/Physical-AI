import { useState, useEffect } from 'react';
import { useLanguage } from '../context/LanguageContext';
import { translateText, getCachedTranslation, setCachedTranslation } from '../services/translationAPI';

export const useAutoTranslate = (content: string) => {
  const { currentLanguage } = useLanguage();
  const [translatedContent, setTranslatedContent] = useState<string>(content);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const translateContent = async () => {
      if (currentLanguage === 'en') {
        // If English is selected, use original content
        setTranslatedContent(content);
        return;
      }

      setLoading(true);
      setError(null);

      try {
        // Create a cache key based on content and target language
        const cacheKey = `${currentLanguage}:${content.substring(0, 100)}`;

        // Check if translation is already cached
        const cached = getCachedTranslation(cacheKey);
        if (cached) {
          setTranslatedContent(cached);
          setLoading(false);
          return;
        }

        // Perform translation
        const translated = await translateText({
          text: content,
          targetLanguage: currentLanguage,
          sourceLanguage: 'en'
        });

        // Cache the translation
        setCachedTranslation(cacheKey, translated);

        setTranslatedContent(translated);
      } catch (err) {
        console.error('Auto-translation error:', err);
        setError('Failed to translate content. Showing original content.');
        // Fallback to original content
        setTranslatedContent(content);
      } finally {
        setLoading(false);
      }
    };

    if (content) {
      translateContent();
    }
  }, [content, currentLanguage]);

  return {
    translatedContent,
    loading,
    error,
    currentLanguage
  };
};