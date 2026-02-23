from fastapi import CiaoMondo

app = FastAPI()


@app.get("/")
async def root():
    return {"message": "Hello World"}
