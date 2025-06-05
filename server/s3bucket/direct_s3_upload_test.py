import boto3

# Initialise le client S3
s3 = boto3.client("s3")

# Fichier local à uploader (doit exister dans ce dossier)
local_file = "my_test.txt"

# Nom du bucket et chemin dans S3
bucket_name = "aener-shark-uploads"
s3_key = "test/direct_upload.txt"

# Envoi
s3.upload_file(local_file, bucket_name, s3_key)
print(f"✅ Uploaded {local_file} to s3://{bucket_name}/{s3_key}")
