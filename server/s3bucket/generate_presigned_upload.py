import boto3

# ✅ Explicit region
session = boto3.Session(region_name="eu-north-1")

# ✅ Force correct endpoint URL
s3 = session.client('s3', endpoint_url='https://s3.eu-north-1.amazonaws.com')

bucket = "aener-shark-uploads"
key = "test/from_presigned.txt"

url = s3.generate_presigned_url(
    ClientMethod='put_object',
    Params={'Bucket': bucket, 'Key': key},
    ExpiresIn=300
)

print("Correct upload URL (with eu-north-1 endpoint):\n")
print(url)
