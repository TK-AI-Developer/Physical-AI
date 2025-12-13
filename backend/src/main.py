from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from .api import chapters
from .logging import logger

app = FastAPI()

app.include_router(chapters.router)

@app.exception_handler(Exception)
async def generic_exception_handler(request: Request, exc: Exception):
    logger.error(f"An unexpected error occurred: {exc}")
    return JSONResponse(
        status_code=500,
        content={"message": "Internal server error"},
    )

@app.get("/")
def read_root():
    return {"Hello": "World"}
