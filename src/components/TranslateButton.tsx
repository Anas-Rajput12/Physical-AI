import React, { useState } from 'react';
import { useLanguage } from '../context/LanguageContext';
import { translateText, getCachedTranslation, setCachedTranslation } from '../services/translationAPI';

interface TranslateButtonProps {
  content: string;
  onTranslation: (translatedContent: string) => void;
  className?: string;
}

const TranslateButton: React.FC<TranslateButtonProps> = ({
  content,
  onTranslation,
  className = ''
}) => {
  const { currentLanguage } = useLanguage();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleTranslate = async () => {
    if (currentLanguage === 'en') {
      // If English is selected, show original content
      onTranslation(content);
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
        onTranslation(cached);
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

      // Update the content with translated text
      onTranslation(translated);
    } catch (err) {
      console.error('Translation error:', err);
      setError('Failed to translate content. Please try again.');
      // Fallback to original content
      onTranslation(content);
    } finally {
      setLoading(false);
    }
  };

  if (currentLanguage === 'en') {
    return null; // Don't show button when English is selected
  }

  return (
    <button
      onClick={handleTranslate}
      disabled={loading}
      className={`button button--secondary ${className}`}
      style={{
        display: 'flex',
        alignItems: 'center',
        gap: '0.5rem',
        padding: '0.5rem 1rem',
        fontSize: '0.875rem',
      }}
    >
      {loading ? (
        <>
          <span
            className="button__loader"
            style={{
              width: '12px',
              height: '12px',
              border: '2px solid transparent',
              borderTop: '2px solid currentColor',
              borderRadius: '50%',
              animation: 'spin 1s linear infinite',
            }}
          ></span>
          Translating...
        </>
      ) : (
        <>
          <span>🌐</span>
          Translate Chapter
        </>
      )}
    </button>
  );
};

export default TranslateButton;