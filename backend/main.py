from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import uvicorn
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import API routes
from api.chat import router as chat_router
from api.documents import router as documents_router

# Create FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG API",
    description="REST API for RAG-based Q&A system for the Physical AI & Humanoid Robotics book",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(chat_router, prefix="/api/v1", tags=["chat"])
app.include_router(documents_router, prefix="/api/v1", tags=["documents"])

@app.get("/")
def read_root():
    return {
        "message": "Physical AI & Humanoid Robotics RAG API",
        "version": "1.0.0",
        "endpoints": [
            "/api/v1/chat",
            "/api/v1/documents",
            "/docs"
        ]
    }

@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "Physical AI & Humanoid Robotics RAG API"}

if __name__ == "__main__":
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)