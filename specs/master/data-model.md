# Data Model

The data model for the physical AI book is designed to be simple and flexible, allowing for easy content creation and management. The primary entities are `Chapter` and `Module`.

## Entities

### Chapter

A `Chapter` represents a main section of the book.

- **`title`**: (String) The title of the chapter.
- **`slug`**: (String) The URL-friendly version of the title.
- **`summary`**: (String) A brief overview of the chapter's content.
- **`modules`**: (Array of `Module`) The modules that belong to this chapter.

### Module

A `Module` represents a specific topic within a chapter.

- **`title`**: (String) The title of the module.
- **`slug`**: (String) The URL-friendly version of the title.
- **`content`**: (Markdown) The main content of the module.
- **`example`**: (Code) A code example related to the module's content.
- **`exercise`**: (String) A practical exercise for the reader.

## Relationships

- A `Chapter` can have multiple `Module`s.
- Each `Module` belongs to a single `Chapter`.

## State Transitions

The content is static and does not have state transitions in the traditional sense. The book's content will be updated through a Git-based workflow.
