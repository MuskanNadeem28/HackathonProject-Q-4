from fastapi import APIRouter, HTTPException, BackgroundTasks
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from services.rag_service import RAGService
from models.chat_models import ChatRequest, ChatResponse, ChatMessage, SourceDocument

router = APIRouter()
rag_service = RAGService()

class ChatRequest(BaseModel):
    query: str
    context_window: Optional[str] = None  # Optional context window (entire book or selected text)
    max_tokens: Optional[int] = 500
    temperature: Optional[float] = 0.7

class ChatResponse(BaseModel):
    response: str
    sources: List[SourceDocument]
    query: str
    context_window: Optional[str]

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(chat_request: ChatRequest):
    """
    Main chat endpoint for the RAG system
    """
    try:
        # Process the chat request using the RAG service
        response, sources = await rag_service.process_query(
            query=chat_request.query,
            context_window=chat_request.context_window,
            max_tokens=chat_request.max_tokens,
            temperature=chat_request.temperature
        )

        return ChatResponse(
            response=response,
            sources=sources,
            query=chat_request.query,
            context_window=chat_request.context_window
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@router.post("/chat-stream")
async def chat_stream_endpoint(chat_request: ChatRequest):
    """
    Streaming chat endpoint for the RAG system
    """
    try:
        # This would implement server-sent events for streaming responses
        # Implementation would depend on specific streaming requirements
        raise HTTPException(status_code=501, detail="Streaming not yet implemented")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing stream query: {str(e)}")