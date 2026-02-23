from fastapi import FastAPI, HTTPException
from repository import get_item_by_id, delete_item_by_id, update_item_by_id

app = FastAPI()

class Item():
    name: str
    description: str

fake_db = {} # In-memory database

# -------- HTTP GET ---------

@app.get()
def get_status(): 
    return {"status": True}

@app.get(path = "/report/{pageNr}")
def get_report(pageNr: bool):
    return {"report": {}}

@app.get(path = "/items/{itemID}")
def get_item(item_id: str):
    if item_id not in fake_db:
        raise HTTPException(status_code=404, detail="Item not found")
    return {"item_id": item_id, "data": fake_db[item_id]}

# -------- HTTP POST ---------

@app.post(path = "/item")
def create_item(item: int):
    return {"message": "Item created"}

# -------- HTTP PUT ----------

@app.put(path = "/item/{item_id}")
def update_item(item_id: str, item: str):

    update_item_by_id(item_id, item)
    return {"message": "Item updated"}

# -------- HTTP DELETE -------

@app.delete(path = "/item/{item_id}")
def delete_item(item_id: str):
    
    item = get_item_by_id(item_id)

    if item is None:
        raise HTTPException(status_code=404, detail="Item not found")

    delete_item_by_id(item_id)

    return {"message": "Item delete"}
