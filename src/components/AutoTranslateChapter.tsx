import React, { useState, useEffect } from 'react';
import TranslateButton from '../components/TranslateButton';
import { useAutoTranslate } from '../hooks/useAutoTranslate';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';

interface AutoTranslateChapterProps {
  children: string;
  title?: string;
}

const AutoTranslateChapter: React.FC<AutoTranslateChapterProps> = ({ children, title }) => {
  const [originalContent, setOriginalContent] = useState<string>('');
  const { translatedContent, loading, error, currentLanguage } = useAutoTranslate(originalContent);

  useEffect(() => {
    if (typeof children === 'string') {
      setOriginalContent(children);
    }
  }, [children]);

  return (
    <div className="auto-translate-chapter">
      {title && <h1>{title}</h1>}

      <div style={{ marginBottom: '1rem' }}>
        <TranslateButton
          content={originalContent}
          onTranslation={setOriginalContent}
        />
      </div>

      {loading && (
        <div className="alert alert--info">
          Translating content...
        </div>
      )}

      {error && currentLanguage !== 'en' && (
        <div className="alert alert--danger">
          {error}
        </div>
      )}

      <div className="chapter-content">
        <ReactMarkdown
          remarkPlugins={[remarkGfm]}
          components={{
            // Custom components to handle specific elements
            h1: (props) => <h1 {...props} />,
            h2: (props) => <h2 {...props} />,
            h3: (props) => <h3 {...props} />,
            p: (props) => <p {...props} />,
            ul: (props) => <ul {...props} />,
            ol: (props) => <ol {...props} />,
            li: (props) => <li {...props} />,
            strong: (props) => <strong {...props} />,
            em: (props) => <em {...props} />,
            code: (props) => <code {...props} />,
            pre: (props) => <pre {...props} />,
          }}
        >
          {currentLanguage === 'en' ? originalContent : translatedContent}
        </ReactMarkdown>
      </div>

      <style jsx>{`
        .auto-translate-chapter {
          position: relative;
        }

        .chapter-content {
          min-height: 100px;
        }
      `}</style>
    </div>
  );
};

export default AutoTranslateChapter;