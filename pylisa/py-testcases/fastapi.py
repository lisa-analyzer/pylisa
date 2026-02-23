from fastapi import APIRouter, FastAPI, WebSocket
import gradio as gr
from pydantic import BaseModel
version = "v1"
app = FastAPI(root_path="root" + "/" + version)
router = APIRouter(prefix="/router")
app.get("/ciao")
version()
class C:
    x = 10

f = C
C = 3
f()
@router.get("/items/")
def read_items():
    return
#app.include_router(router)

x = 10
class Numbers(BaseModel):
    num1: float
    num2: float
def f():
    pass
def g():
    return f
@app.post("/add")
async def add_numbers(numbers: Numbers):
    result = numbers.num1 + numbers.num2
    return {"sum": result}
# g()(app.post("/add")(app.get("/add")(add_numbers)))

def greet(name):
    return f"Hello, {name}!"
app.include_router(router)
gradio_app = gr.Interface(fn=greet, inputs="text", outputs="text")

app = gr.mount_gradio_app(app, gradio_app, path="/gradio")

@app.get("/")
async def read_root():
    return {"message": "Welcome to the FastAPI and Gradio app!"}




if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)