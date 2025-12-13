import os
import frontmatter
from typing import List
from ..models.schemas import Chapter, Module

CONTENT_DIR = "content"

def get_chapters() -> List[Chapter]:
    chapters = []
    for chapter_dir in os.listdir(CONTENT_DIR):
        chapter_path = os.path.join(CONTENT_DIR, chapter_dir)
        if os.path.isdir(chapter_path):
            modules = []
            for module_file in os.listdir(chapter_path):
                if module_file.endswith(".md"):
                    module_path = os.path.join(chapter_path, module_file)
                    with open(module_path, "r") as f:
                        post = frontmatter.load(f)
                        module = Module(
                            title=post["title"],
                            slug=post["slug"],
                            content=post.content,
                            example=post["example"],
                            exercise=post["exercise"],
                        )
                        modules.append(module)
            chapter = Chapter(
                title=chapter_dir.replace("-", " ").title(),
                slug=chapter_dir,
                summary=f"This is {chapter_dir}",
                modules=modules,
            )
            chapters.append(chapter)
    return chapters

def get_chapter(slug: str) -> Chapter:
    chapters = get_chapters()
    for chapter in chapters:
        if chapter.slug == slug:
            return chapter
    return None
