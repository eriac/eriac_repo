# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# PDX-License-Identifier: MIT-0 (For details, see https://github.com/awsdocs/amazon-rekognition-developer-guide/blob/master/LICENSE-SAMPLECODE.)

import boto3
import os
import sys

# サービスを利用するための識別情報(iniファイルの中身）を読み込む
access_key = os.getenv('AWS_ACCESS_KEY_ID')
secret_key = os.getenv('AWS_SECRET_ACCESS_KEY')
region = "ap-northeast-1"


def list_faces_in_collection(session, collection_id):
    maxResults = 2
    faces_count = 0
    tokens = True

    client = session.client('rekognition')
    response = client.list_faces(CollectionId=collection_id,
                                 MaxResults=maxResults)

    print('Faces in collection ' + collection_id)

    while tokens:

        faces = response['Faces']

        for face in faces:
            print(face)
            faces_count += 1
        if 'NextToken' in response:
            nextToken = response['NextToken']
            response = client.list_faces(CollectionId=collection_id,
                                         NextToken=nextToken, MaxResults=maxResults)
        else:
            tokens = False
    return faces_count

def upload_image(session, bucket, filename):
    s3 = session.client("s3")
    
    # ファイルを読み込む
    with open(filename, "rb") as f:
        # 読み込んだファイルをS3サービスにアップロード
        s3.put_object(Bucket=bucket, Key=filename, Body=f)

def search_faces(session, bucket, filename, collection_id):
    threshold = 50
    maxFaces=3

    client=session.client('rekognition')
    response=client.search_faces_by_image(CollectionId=collection_id,
                                Image={'S3Object':{'Bucket':bucket,'Name':filename}},
                                FaceMatchThreshold=threshold,
                                MaxFaces=maxFaces)

                                
    faceMatches=response['FaceMatches']
    print ('Matching faces')
    for match in faceMatches:
            print ('FaceId:' + match['Face']['FaceId'])
            print ('Similarity: ' + "{:.2f}".format(match['Similarity']) + "%")
            print
    print()
    print(response)

def main():
    # サービスへの接続情報を取得する
    session = boto3.Session(
        aws_access_key_id=access_key, aws_secret_access_key=secret_key, region_name=region
    )

    collection_id = 'collection-id'
    faces_count = list_faces_in_collection(session, collection_id)
    print("faces count: " + str(faces_count))


    args = sys.argv
    assert 2 <= len(args)
    filename = args[1]
    print('check '+filename)
    bucket = "rekognition-test-765157193081-ap-northeast-1"

    upload_image(session, bucket, filename)
    search_faces(session, bucket, filename, collection_id)

if __name__ == "__main__":
    main()