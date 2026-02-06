import React from 'react';
import RAGChatbot from '../components/RAGChatbot';

interface RootProps {
  children: React.ReactNode;
}

// This component wraps the entire site and adds the chatbot to all pages
export default function Root({ children }: RootProps): JSX.Element {
  return (
    <>
      {children}
      <RAGChatbot />
    </>
  );
}
