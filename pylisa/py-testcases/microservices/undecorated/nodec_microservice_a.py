from fastapi import FastAPI, APIRouter, HTTPException
from repository import get_item_by_id, delete_item_by_id, update_item_by_id

app = FastAPI()
router = APIRouter()

class Item():
    name: str
    description: str

fake_db = {}

# -------- HTTP GET ---------

def get_status(): 
    return {"status": True}

router.add_api_route(endpoint=get_status, methods=["GET"])

def get_report(pageNr: bool):
    return {"report": {}}

router.add_api_route(path = "/report/{pageNr}", endpoint=get_report, methods=["GET"])

def get_item(item_id: str):
    if item_id not in fake_db:
        raise HTTPException(status_code=404, detail="Item not found")
    return {"item_id": item_id, "data": fake_db[item_id]}

router.add_api_route(path = "/items/{itemID}", endpoint=get_item, methods=["GET"])

# -------- HTTP POST ---------

def create_item(item: int):
    return {"message": "Item created"}

router.add_api_route(path = "/item", endpoint=create_item, methods=["POST"])

# -------- HTTP PUT ----------

def update_item(item_id: str, item: str):

    update_item_by_id(item_id, item)
    return {"message": "Item updated"}

router.add_api_route(path = "/item/{item_id}", endpoint=update_item, methods=["PUT"])

# -------- HTTP DELETE -------

def delete_item(item_id: str):
    
    item = get_item_by_id(item_id)

    if item is None:
        raise HTTPException(status_code=404, detail="Item not found")

    delete_item_by_id(item_id)

    return {"message": "Item delete"}

router.add_api_route(path = "/item/{item_id}", endpoint=delete_item, methods=["DELETE"])

app.include_router(router)