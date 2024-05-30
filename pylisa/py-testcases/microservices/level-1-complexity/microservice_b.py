from fastapi import FastAPI
import requests

app = FastAPI()

@app.put(path = "/manufacturer/{id}")
def update_manufacturer(id: str, name: int):
    return {"status": {}}

def get_report():
    response = requests.get("http://microservice_a:8000/report/3")

def create_quota():
    response = requests.post("http://microservice_c:8000/quota/fik39")