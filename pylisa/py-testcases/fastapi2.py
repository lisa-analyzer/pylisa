import fastapi
#from fastapi import APIRouter, FastAPI, WebSocket
#import gradio as gr
#from pydantic import BaseModel
version = "v1"
g = fastapi.FastAPI
from fastapi import FastAPI, APIRouter
app = g()
app2 = fastapi.FastAPI(root_path="api" + "/" + version)
def f(x):
    y = 3
    return x + y
x = FastAPI(root_path="TEST")

router = fastapi.APIRouter(prefix="api_router_prefix")
@router.get("/foo")
def router_get():
    return

@router.get("/foo2")
def router_get_2():
    return

new_function_name = f
h = new_function_name(10)
app.include_router(router, prefix=h)
@app.get("/foo")
def hh():
    return

@app.post("/bar")
def router_foo():
    return
app2.include_router(router, prefix="bar")