from fastapi import FastAPI

app = FastAPI()


async def root():
    return {"message": "Hello World"}
