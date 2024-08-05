from fastapi import FastAPI
import requests

app = FastAPI()

@app.post(path = "/quota/{name}")
def create_quota(id: str):
    return {"status": {}}

def get_report():
    response = requests.get("http://microservice_a:8000/report/1")

def update_manufacturer():
    response = requests.put("http://microservice_b:8000/manufacturer/ChanVendor")
