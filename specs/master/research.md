# Research

## Primary Dependencies

- **Decision**: FastAPI for the backend, React (Docusaurus) for the frontend, and ROS2 for robotics integration.
- **Rationale**: FastAPI is a modern, fast (high-performance) web framework for building APIs with Python 3.7+ based on standard Python type hints. Docusaurus is a static site generator that helps you ship beautiful, accessible documentation websites fast. ROS2 is the de-facto standard for robotics application development.
- **Alternatives considered**: Flask for the backend, but FastAPI is more modern and has better performance. Vue or Angular for the frontend, but Docusaurus is specialized for documentation websites.

## Storage

- **Decision**: A combination of file-based storage for the book's content (Markdown files) and a simple database (like SQLite or a JSON file) for user-specific data if needed in the future.
- **Rationale**: The primary content of the book is text-based and fits well with a Git-based workflow using Markdown files. A simple database is sufficient for any potential future needs without adding the complexity of a full-fledged database server.
- **Alternatives considered**: A full database like PostgreSQL, but it's overkill for the current scope.

## Testing

- **Decision**: Pytest for the backend and Jest for the frontend.
- **Rationale**: Pytest is a mature, feature-rich testing framework for Python. Jest is the standard for testing React applications.
- **Alternatives considered**: Unittest for the backend, but Pytest is generally considered easier to use.

## Target Platform

- **Decision**: The backend will be a Linux server (or a container), and the frontend will be a static website deployed on GitHub Pages.
- **Rationale**: This is a common and cost-effective setup for web applications.
- **Alternatives considered**: A more complex cloud setup, but that's not necessary for the current scope.

## Project Type

- **Decision**: A web application with a Jamstack architecture (JavaScript, APIs, and Markup).
- **Rationale**: This architecture is well-suited for content-driven websites like the physical AI book.
- **Alternatives considered**: A monolithic application, but separating the frontend and backend is a more flexible and scalable approach.

## Performance Goals

- **Decision**: p95 latency for API calls under 200ms.
- **Rationale**: This is a reasonable performance target for a web application of this type.
- **Alternatives considered**: Stricter goals, but they are not necessary at this stage.

## Constraints

- **Decision**: The book must have a minimum total length of 20,000–35,000 words. There must be at least one working demo (simulation is acceptable). All images and figures must include alt text for accessibility.
- **Rationale**: These constraints are taken from the project's constitution.
- **Alternatives considered**: None.

## Scale/Scope

- **Decision**: The initial scope is to build the book's platform and the first few chapters. The target audience is students and hobbyists.
- **Rationale**: This is a manageable scope for the initial version of the project.
- **Alternatives considered**: A more ambitious scope, but it's better to start small and iterate.
