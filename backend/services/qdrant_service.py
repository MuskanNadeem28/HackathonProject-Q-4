from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional, Dict, Any
from config.settings import settings
import openai
from sentence_transformers import SentenceTransformer
import logging

logger = logging.getLogger(__name__)

class QdrantService:
    def __init__(self):
        # Initialize Qdrant client
        if settings.QDRANT_URL and settings.QDRANT_API_KEY:
            self.client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                prefer_grpc=True
            )
        else:
            # For development, you can use local Qdrant
            self.client = QdrantClient(host="localhost", port=6333)

        # Initialize embedding model
        self.embedding_model = SentenceTransformer('all-MiniLM-L6-v2')

        # Set collection name
        self.collection_name = settings.QDRANT_COLLECTION_NAME or "physical_ai_robotics"

    def _get_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for the given text
        """
        embedding = self.embedding_model.encode(text)
        return embedding.tolist()

    async def search(
        self,
        query: str,
        limit: int = 5,
        filters: Optional[Dict[str, Any]] = None,
        context_window: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for documents in the Qdrant collection
        """
        try:
            query_vector = self._get_embedding(query)

            # Build search filters
            search_filter = models.Filter()
            if filters:
                filter_conditions = []
                for key, value in filters.items():
                    filter_conditions.append(
                        models.FieldCondition(
                            key=f"metadata.{key}",
                            match=models.MatchValue(value=value)
                        )
                    )
                search_filter = models.Filter(must=filter_conditions)

            # Add context window filter if specified
            if context_window:
                if context_window.startswith("module_"):
                    module_name = context_window.replace("module_", "")
                    search_filter.must.append(
                        models.FieldCondition(
                            key="metadata.module",
                            match=models.MatchValue(value=module_name)
                        )
                    )

            # Perform search
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                query_filter=search_filter,
                limit=limit,
                with_payload=True
            )

            # Format results
            results = []
            for hit in search_results:
                result = {
                    "id": hit.id,
                    "content": hit.payload.get("content", ""),
                    "title": hit.payload.get("title", ""),
                    "source": hit.payload.get("source", ""),
                    "score": hit.score,
                    "metadata": hit.payload.get("metadata", {})
                }
                results.append(result)

            return results

        except Exception as e:
            logger.error(f"Error searching in Qdrant: {str(e)}")
            return []

    async def get_document_by_id(self, doc_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a specific document by ID
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[doc_id],
                with_payload=True
            )

            if not records:
                return None

            record = records[0]
            return {
                "id": record.id,
                "content": record.payload.get("content", ""),
                "title": record.payload.get("title", ""),
                "source": record.payload.get("source", ""),
                "metadata": record.payload.get("metadata", {})
            }

        except Exception as e:
            logger.error(f"Error retrieving document by ID: {str(e)}")
            return None

    async def get_available_modules(self) -> List[str]:
        """
        Get list of available modules in the knowledge base
        """
        try:
            # Get all unique module names from the collection
            # This is a simplified approach - in a real implementation you might want to use Qdrant's facet search
            all_records = self.client.scroll(
                collection_name=self.collection_name,
                limit=1000,  # Adjust as needed
                with_payload=True
            )

            modules = set()
            for record, _ in all_records:
                module = record.payload.get("metadata", {}).get("module")
                if module:
                    modules.add(module)

            return list(modules)

        except Exception as e:
            logger.error(f"Error getting available modules: {str(e)}")
            return []

    async def insert_documents(self, documents: List[Dict[str, Any]]):
        """
        Insert documents into the Qdrant collection
        """
        try:
            points = []
            for i, doc in enumerate(documents):
                # Generate embedding for the document content
                embedding = self._get_embedding(doc.get("content", ""))

                point = models.PointStruct(
                    id=i,
                    vector=embedding,
                    payload={
                        "content": doc.get("content", ""),
                        "title": doc.get("title", ""),
                        "source": doc.get("source", ""),
                        "metadata": doc.get("metadata", {})
                    }
                )
                points.append(point)

            # Upload points to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

        except Exception as e:
            logger.error(f"Error inserting documents: {str(e)}")