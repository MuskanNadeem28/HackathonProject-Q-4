import asyncio
from typing import List, Tuple, Optional
from models.chat_models import ChatResponse, SourceDocument
from services.qdrant_service import QdrantService
from services.openai_service import OpenAIService
from config.settings import settings

class RAGService:
    def __init__(self):
        self.qdrant_service = QdrantService()
        self.openai_service = OpenAIService()

    async def process_query(
        self,
        query: str,
        context_window: Optional[str] = None,
        max_tokens: int = 500,
        temperature: float = 0.7
    ) -> Tuple[str, List[SourceDocument]]:
        """
        Process a query using the RAG system
        """
        # Step 1: Search for relevant documents in the vector database
        search_results = await self.qdrant_service.search(query, limit=5, context_window=context_window)

        if not search_results:
            # If no relevant documents found, return a default response
            return "I couldn't find any relevant information in the book to answer your question.", []

        # Step 2: Format the context from retrieved documents
        context = self._format_context(search_results)

        # Step 3: Generate response using OpenAI
        response = await self.openai_service.generate_response(
            query=query,
            context=context,
            max_tokens=max_tokens,
            temperature=temperature
        )

        # Convert search results to SourceDocument format
        source_docs = [
            SourceDocument(
                id=doc["id"],
                title=doc["title"],
                content=doc["content"][:200] + "..." if len(doc["content"]) > 200 else doc["content"],  # Truncate content
                source=doc["source"],
                score=doc["score"],
                metadata=doc.get("metadata", {})
            )
            for doc in search_results
        ]

        return response, source_docs

    def _format_context(self, search_results: List[dict]) -> str:
        """
        Format the retrieved documents into a context string for the LLM
        """
        context_parts = []
        for doc in search_results:
            title = doc.get("title", "Untitled")
            content = doc.get("content", "")
            source = doc.get("source", "Unknown")

            context_parts.append(f"Document: {title}\nSource: {source}\nContent: {content}\n---\n")

        return "\n".join(context_parts)

    async def get_answer_with_sources(
        self,
        query: str,
        context_window: Optional[str] = None
    ) -> ChatResponse:
        """
        Get a complete answer with sources for the chat response
        """
        response, sources = await self.process_query(query, context_window)

        return ChatResponse(
            response=response,
            sources=sources,
            query=query,
            context_window=context_window
        )