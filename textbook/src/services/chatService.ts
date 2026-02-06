/**
 * Chat service for communicating with the RAG chatbot API.
 */

const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-api-domain.vercel.app/api/v1'
  : 'http://localhost:8000/api/v1';

export interface Source {
  chapter_slug: string;
  section_title: string;
  chapter_title: string;
  relevance_score: number;
  text_snippet?: string;
}

export interface ChatResponse {
  message_id: string;
  content: string;
  language: string;
  sources: Source[];
  tokens_used: number;
}

export interface Session {
  id: string;
  session_token: string;
  created_at: string;
}

export interface ChatMessage {
  message: string;
  session_id?: string;
  language?: string;
  selected_text?: string;
  chapter_context?: string;
}

class ChatService {
  private baseUrl: string;

  constructor() {
    this.baseUrl = API_BASE_URL;
  }

  async createSession(): Promise<Session> {
    const response = await fetch(`${this.baseUrl}/chat/sessions`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
    });

    if (!response.ok) {
      throw new Error('Failed to create session');
    }

    return response.json();
  }

  async sendMessage(request: ChatMessage): Promise<ChatResponse> {
    const response = await fetch(`${this.baseUrl}/chat/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error('Failed to send message');
    }

    return response.json();
  }

  async getSession(sessionId: string): Promise<{ session: Session; messages: any[] }> {
    const response = await fetch(`${this.baseUrl}/chat/sessions/${sessionId}`);

    if (!response.ok) {
      throw new Error('Failed to get session');
    }

    return response.json();
  }

  async deleteSession(sessionId: string): Promise<void> {
    const response = await fetch(`${this.baseUrl}/chat/sessions/${sessionId}`, {
      method: 'DELETE',
    });

    if (!response.ok) {
      throw new Error('Failed to delete session');
    }
  }

  async getChapterSummary(chapterSlug: string, language: string = 'en'): Promise<{
    chapter_slug: string;
    summary: string;
    language: string;
    tokens_used: number;
  }> {
    const response = await fetch(
      `${this.baseUrl}/chat/summary/${chapterSlug}?language=${language}`
    );

    if (!response.ok) {
      throw new Error('Failed to get chapter summary');
    }

    return response.json();
  }
}

export const chatService = new ChatService();
export default chatService;
