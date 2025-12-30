import React from 'react';
import { useLanguage } from '../context/LanguageContext';

const LanguageSwitcher: React.FC = () => {
  const { currentLanguage, languages, changeLanguage } = useLanguage();

  const handleLanguageChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const selectedLang = e.target.value;
    changeLanguage(selectedLang);
  };

  return (
    <div className="dropdown dropdown--right dropdown--navbar" style={{ marginLeft: '1rem' }}>
      <select
        value={currentLanguage}
        onChange={handleLanguageChange}
        style={{
          padding: '0.25rem 0.5rem',
          borderRadius: '4px',
          border: '1px solid var(--ifm-color-emphasis-300)',
          backgroundColor: 'var(--ifm-background-surface-color)',
          color: 'var(--ifm-font-color-base)',
        }}
      >
        {languages.map((lang) => (
          <option key={lang.code} value={lang.code}>
            {lang.name}
          </option>
        ))}
      </select>
    </div>
  );
};

export default LanguageSwitcher;