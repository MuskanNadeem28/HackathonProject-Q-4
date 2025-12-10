from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from enum import Enum

class MessageRole(str, Enum):
    USER = "user"
    ASSISTANT = "assistant"
    SYSTEM = "system"

class ChatMessage(BaseModel):
    role: MessageRole
    content: str
    timestamp: Optional[str] = None

class SourceDocument(BaseModel):
    id: str
    title: str
    content: str
    source: str
    score: float
    metadata: Dict[str, Any]

class ChatRequest(BaseModel):
    query: str
    context_window: Optional[str] = None  # "full_book", "selected_module", "specific_pages"
    max_tokens: Optional[int] = 500
    temperature: Optional[float] = 0.7
    history: Optional[List[ChatMessage]] = []

class ChatResponse(BaseModel):
    response: str
    sources: List[SourceDocument]
    query: str
    context_window: Optional[str]
    tokens_used: Optional[int] = None