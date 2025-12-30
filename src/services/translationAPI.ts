// For Docusaurus frontend, API key should be configured differently
// We'll check for it in window or provide a method to set it
declare global {
  interface Window {
    OPENROUTER_API_KEY?: string;
  }
}

const OPENROUTER_API_KEY =
  typeof window !== 'undefined' && window.OPENROUTER_API_KEY
    ? window.OPENROUTER_API_KEY
    : (typeof process !== 'undefined' && process.env ? process.env.OPENROUTER_API_KEY || '' : '');

interface TranslationRequest {
  text: string;
  targetLanguage: string;
  sourceLanguage?: string;
}

export const translateText = async ({
  text,
  targetLanguage,
  sourceLanguage = 'en'
}: TranslationRequest): Promise<string> => {
  if (!OPENROUTER_API_KEY) {
    throw new Error('OpenRouter API key is not configured');
  }

  // Map language codes to full names for the prompt
  const languageMap: Record<string, string> = {
    'en': 'English',
    'ur': 'Urdu',
    'sd': 'Sindhi'
  };

  const targetLangName = languageMap[targetLanguage] || targetLanguage;
  const sourceLangName = languageMap[sourceLanguage] || sourceLanguage;

  const response = await fetch('https://openrouter.ai/api/v1/chat/completions', {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${OPENROUTER_API_KEY}`,
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      model: 'google/gemini-pro', // Using Gemini for translation
      messages: [
        {
          role: 'system',
          content: `You are a professional translator. Translate the following text from ${sourceLangName} to ${targetLangName}. Maintain the original meaning, tone, and context. For technical terms related to AI and robotics, use appropriate terminology in the target language. If the target language is Urdu or Sindhi, use appropriate script (Arabic script for both). Return only the translated text without any additional explanations.`
        },
        {
          role: 'user',
          content: text
        }
      ],
      temperature: 0.1, // Low temperature for more consistent translations
    })
  });

  if (!response.ok) {
    throw new Error(`Translation API error: ${response.status} ${response.statusText}`);
  }

  const data = await response.json();
  return data.choices[0].message.content.trim();
};

// Simple in-memory cache for translations
const translationCache = new Map<string, string>();

export const getCachedTranslation = (key: string): string | undefined => {
  return translationCache.get(key);
};

export const setCachedTranslation = (key: string, translation: string): void => {
  translationCache.set(key, translation);
  // Limit cache size to prevent memory issues
  if (translationCache.size > 1000) {
    const firstKey = translationCache.keys().next().value;
    translationCache.delete(firstKey);
  }
};