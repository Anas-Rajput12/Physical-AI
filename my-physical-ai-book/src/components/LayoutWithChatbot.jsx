import React from 'react';
import Layout from '@theme/Layout';
import RagChatbot from '@site/src/components/RagChatbot';

/**
 * Custom layout wrapper that includes the RAG chatbot
 */
export default function LayoutWithChatbot(props) {
  // Use environment variable for backend URL
  const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';

  return (
    <Layout {...props}>
      {props.children}
      <RagChatbot backendUrl={backendUrl} />
    </Layout>
  );
}