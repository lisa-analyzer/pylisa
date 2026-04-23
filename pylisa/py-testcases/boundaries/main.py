from fastapi import FastAPI
import boto3

app = FastAPI(root_path="MySvc")
s3 = boto3.client("s3")

@app.get("/items")
def get_items():
    return s3.get_object(Bucket="my-bucket", Key="items.json")

@app.post("/items")
def create_item():
    s3.put_object(Bucket="my-bucket", Key="items2.json", Body="{}")
