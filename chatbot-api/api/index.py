import os

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
async def root():
    return {
        "name": "Physical AI Textbook RAG Chatbot",
        "version": "1.0.0",
        "status": "running",
    }


@app.get("/api/health")
async def health():
    return {"status": "ok"}


@app.post("/api/chat")
async def chat_endpoint(request: dict):
    """Chat endpoint."""
    try:
        from openai import OpenAI

        api_key = os.getenv("OPENAI_API_KEY", "")
        if not api_key:
            return {"message": "OpenAI API key not configured", "sources": []}

        client = OpenAI(api_key=api_key)

        message = request.get("message", "")

        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {
                    "role": "system",
                    "content": "You are a helpful assistant for a Physical AI & Humanoid Robotics textbook. Answer questions about ROS 2, robotics, simulation, and AI.",
                },
                {"role": "user", "content": message},
            ],
            temperature=0.7,
            max_tokens=500,
        )

        return {"message": response.choices[0].message.content, "sources": []}
    except Exception as e:
        return {"message": f"Error: {str(e)}", "sources": [], "error": str(e)}


@app.post("/api/chat/selected-text")
async def selected_text_endpoint(request: dict):
    """Answer questions about selected text."""
    try:
        from openai import OpenAI

        api_key = os.getenv("OPENAI_API_KEY", "")
        if not api_key:
            return {"message": "OpenAI API key not configured", "sources": []}

        client = OpenAI(api_key=api_key)

        selected_text = request.get("selected_text", "")
        question = request.get("question", "Please explain this text.")

        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {
                    "role": "system",
                    "content": "You are a helpful assistant for a Physical AI & Humanoid Robotics textbook. Explain the selected text clearly.",
                },
                {
                    "role": "user",
                    "content": f"Selected text: {selected_text}\n\nQuestion: {question}",
                },
            ],
            temperature=0.7,
            max_tokens=500,
        )

        return {"message": response.choices[0].message.content, "sources": []}
    except Exception as e:
        return {"message": f"Error: {str(e)}", "sources": [], "error": str(e)}
