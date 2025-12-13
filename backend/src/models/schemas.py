from pydantic import BaseModel
from typing import List

class Module(BaseModel):
    title: str
    slug: str
    content: str
    example: str
    exercise: str

class Chapter(BaseModel):
    title: str
    slug: str
    summary: str
    modules: List[Module]
