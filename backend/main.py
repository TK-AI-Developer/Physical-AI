from fastapi import FastAPI, Request
from pydantic import BaseModel

app = FastAPI()

# --------------------------
# MCP Context 7 (User Memory)
# --------------------------

class MCPContext7:
    def __init__(self):
        self.sessions = {}  # All users ka data memory me

    def get_context(self, user_id):
        if user_id not in self.sessions:
            self.sessions[user_id] = {
                "current_module": None,
                "current_chapter": None,
                "personalization": {},
                "rag_history": []
            }
        return self.sessions[user_id]

# global context manager
context_manager = MCPContext7()


# --------------------------
# Request Body Model (FIX)
# --------------------------

class AskRequest(BaseModel):
    user_id: str
    question: str


# --------------------------
# /ask API
# --------------------------

@app.post("/ask")
async def ask_ai(body: AskRequest):

    # Extract data safely
    user_id = body.user_id
    question = body.question

    # get context of the user
    user_context = context_manager.get_context(user_id)

    # store question in user memory (RAG-like)
    user_context["rag_history"].append(question)

    # Temporary AI response
    answer = (
        f"AI Answer for: {question}\n"
        f"(Current Module: {user_context['current_module']})"
    )

    return {"answer": answer}
