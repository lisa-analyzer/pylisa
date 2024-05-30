import requests

def getItems():
    response = requests.get("http://microservice_a:8000/items/1")

    if response.status_code == 200:
        return response.json()
    else:
        return None

items_data = getItems()
print(items_data)