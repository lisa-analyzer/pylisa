import boto3

s3 = boto3.client("s3")

s3.delete_object(
    Bucket="my-example-bucket-123",
    Key="uploaded_file.txt"
)