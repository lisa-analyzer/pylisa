import boto3
from fastapi import FastAPI

app = FastAPI()
s3 = boto3.client("s3")
s3.delete_object(Bucket="my-example-bucket-123", Key="test.txt")

@app.delete("/file")
async def delete_file(filename: str):
    pass

@app.post("/upload")
async def upload_file(filename: str, data: bytes):
    pass
