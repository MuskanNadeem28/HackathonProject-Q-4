from typing import List, Optional, Dict, Any
from models.chat_models import DocumentResponse
from services.qdrant_service import QdrantService

class DocumentService:
    def __init__(self):
        self.qdrant_service = QdrantService()

    async def search_documents(
        self,
        query: str,
        limit: int = 10,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[DocumentResponse]:
        """
        Search for documents in the knowledge base
        """
        search_results = await self.qdrant_service.search(query, limit=limit, filters=filters)

        documents = []
        for doc in search_results:
            documents.append(
                DocumentResponse(
                    id=doc["id"],
                    title=doc["title"],
                    content=doc["content"][:200] + "..." if len(doc["content"]) > 200 else doc["content"],
                    source=doc["source"],
                    metadata=doc.get("metadata", {})
                )
            )

        return documents

    async def get_available_modules(self) -> List[str]:
        """
        Get list of available modules in the knowledge base
        """
        return await self.qdrant_service.get_available_modules()

    async def get_document_by_id(self, doc_id: str) -> Optional[DocumentResponse]:
        """
        Get a specific document by ID
        """
        doc = await self.qdrant_service.get_document_by_id(doc_id)

        if not doc:
            return None

        return DocumentResponse(
            id=doc["id"],
            title=doc["title"],
            content=doc["content"],
            source=doc["source"],
            metadata=doc.get("metadata", {})
        )