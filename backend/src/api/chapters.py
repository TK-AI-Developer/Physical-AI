from fastapi import APIRouter, HTTPException
from ..services import content
from ..models.schemas import Chapter
from typing import List

router = APIRouter()

@router.get("/chapters", response_model=List[Chapter])
def get_chapters():
    return content.get_chapters()

@router.get("/chapters/{slug}", response_model=Chapter)
def get_chapter(slug: str):
    chapter = content.get_chapter(slug)
    if not chapter:
        raise HTTPException(status_code=404, detail="Chapter not found")
    return chapter
