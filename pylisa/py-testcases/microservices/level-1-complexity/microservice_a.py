from fastapi import FastAPI
from repository import delete_item_by_id
import requests

app = FastAPI()

@app.get(path = "/report/{pageNr}")
def get_report(pageNr: bool):
    return {"report": {}}

@app.get()
def blank():
    return {"status": True}

@app.delete(path = "/item/{item_id}")
def delete_item(item_id: str):
    delete_item_by_id(item_id)

    return {"message": "Item delete"}

def get_manufacturer():
    response = requests.put("http://microservice_b:8000/manufacturer/JoeVendor")

def create_quota():
    response = requests.post("http://microservice_c:8000/quota/nau16")