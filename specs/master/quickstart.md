# Quickstart

This guide will help you get the project up and running on your local machine.

## Prerequisites

- Python 3.11 or later
- Node.js 16.0 or later
- ROS2 Foxy Fitzroy or later

## Installation

1.  **Clone the repository:**

    ```bash
    git clone https://github.com/your-username/physical-ai-book.git
    cd physical-ai-book
    ```

2.  **Set up the backend:**

    ```bash
    cd backend
    python -m venv venv
    source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
    pip install -r requirements.txt
    ```

3.  **Set up the frontend:**

    ```bash
    cd ../frontend
    npm install
    ```

## Running the application

1.  **Start the backend server:**

    ```bash
    cd backend
    uvicorn main:app --reload
    ```

2.  **Start the frontend development server:**

    ```bash
    cd frontend
    npm start
    ```

The application will be available at `http://localhost:3000`.
