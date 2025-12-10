from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel
from typing import List, Optional
from services.document_service import DocumentService

router = APIRouter()
document_service = DocumentService()

class DocumentSearchRequest(BaseModel):
    query: str
    limit: Optional[int] = 10
    filters: Optional[dict] = {}

class DocumentResponse(BaseModel):
    id: str
    title: str
    content: str
    source: str
    metadata: dict

@router.get("/documents/search", response_model=List[DocumentResponse])
async def search_documents(
    query: str = Query(..., min_length=1, max_length=500),
    limit: int = Query(10, ge=1, le=100),
    module: Optional[str] = Query(None)
):
    """
    Search for documents in the knowledge base
    """
    try:
        filters = {}
        if module:
            filters["module"] = module

        results = await document_service.search_documents(query, limit, filters)
        return results
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error searching documents: {str(e)}")

@router.get("/documents/modules", response_model=List[str])
async def get_modules():
    """
    Get list of available modules in the knowledge base
    """
    try:
        modules = await document_service.get_available_modules()
        return modules
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving modules: {str(e)}")

@router.get("/documents/{doc_id}", response_model=DocumentResponse)
async def get_document(doc_id: str):
    """
    Get a specific document by ID
    """
    try:
        document = await document_service.get_document_by_id(doc_id)
        if not document:
            raise HTTPException(status_code=404, detail="Document not found")
        return document
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving document: {str(e)}")