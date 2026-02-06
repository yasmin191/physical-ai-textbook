import React from "react";
import RAGChatbot from "../components/RAGChatbot";
import TranslateButton from "../components/TranslateButton";

interface RootProps {
  children: React.ReactNode;
}

// This component wraps the entire site and adds the chatbot and translate button to all pages
export default function Root({ children }: RootProps): JSX.Element {
  return (
    <>
      {children}
      <div
        style={{
          position: "fixed",
          top: "70px",
          right: "20px",
          zIndex: 100,
        }}
      >
        <TranslateButton />
      </div>
      <RAGChatbot />
    </>
  );
}
