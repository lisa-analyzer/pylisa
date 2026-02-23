import fastapi
#from fastapi import APIRouter, FastAPI, WebSocket
#import gradio as gr
#from pydantic import BaseModel
version = "v1"
g = fastapi.FastAPI
app = g()
app2 = fastapi.FastAPI(root_path="api" + "/" + version)
def f(x):
    y = 3
    return x + y

router = fastapi.APIRouter(prefix="api_router_prefix")
@router.get("/foo")
def router_get():
    return

new_function_name = f
h = new_function_name(10)
app.include_router(router)
@app.get("/foo")
def hh():
    return