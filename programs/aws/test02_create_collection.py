# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# PDX-License-Identifier: MIT-0 (For details, see https://github.com/awsdocs/amazon-rekognition-developer-guide/blob/master/LICENSE-SAMPLECODE.)

import boto3
import os

# サービスを利用するための識別情報(iniファイルの中身）を読み込む
access_key = os.getenv('AWS_ACCESS_KEY_ID')
secret_key = os.getenv('AWS_SECRET_ACCESS_KEY')
region = "ap-northeast-1"

def create_collection(session, collection_id):
    client = session.client('rekognition')

    # Create a collection
    print('Creating collection:' + collection_id)
    response = client.create_collection(CollectionId=collection_id)
    print('Collection ARN: ' + response['CollectionArn'])
    print('Status code: ' + str(response['StatusCode']))
    print('Done...')

def main():
    # サービスへの接続情報を取得する
    session = boto3.Session(
        aws_access_key_id=access_key, aws_secret_access_key=secret_key, region_name=region
    )

    collection_id = "collection-id"
    create_collection(session, collection_id)

if __name__ == "__main__":
    main()