import boto3
import sys
import os

# サービスを利用するための識別情報(iniファイルの中身）を読み込む
access_key = os.getenv('AWS_ACCESS_KEY_ID')
secret_key = os.getenv('AWS_SECRET_ACCESS_KEY')

def upload_image(session, bucket, filename):
    s3 = session.client("s3")
    
    # ファイルを読み込む
    with open(filename, "rb") as f:
        # 読み込んだファイルをS3サービスにアップロード
        s3.put_object(Bucket=bucket, Key=filename, Body=f)


def add_faces_to_collection(session, bucket, photo, collection_id):
    client = session.client('rekognition')

    response = client.index_faces(CollectionId=collection_id,
                                  Image={'S3Object': {'Bucket': bucket, 'Name': photo}},
                                  ExternalImageId=photo,
                                  MaxFaces=1,
                                  QualityFilter="AUTO",
                                  DetectionAttributes=['ALL'])

    print('Results for ' + photo)
    print('Faces indexed:')
    for faceRecord in response['FaceRecords']:
        print('  Face ID: ' + faceRecord['Face']['FaceId'])
        print('  Location: {}'.format(faceRecord['Face']['BoundingBox']))

    print('Faces not indexed:')
    for unindexedFace in response['UnindexedFaces']:
        print(' Location: {}'.format(unindexedFace['FaceDetail']['BoundingBox']))
        print(' Reasons:')
        for reason in unindexedFace['Reasons']:
            print('   ' + reason)
    return len(response['FaceRecords'])

def main():
    args = sys.argv
    assert 2 <= len(args)
    
    filename = args[1]
    print('add '+filename)

    bucket = "rekognition-test-765157193081-ap-northeast-1"
    collection_id = 'collection-id'
    photo = filename
    region = "ap-northeast-1"

    # サービスへの接続情報を取得する
    session = boto3.Session(
        aws_access_key_id=access_key, aws_secret_access_key=secret_key, region_name=region
    )

    upload_image(session, bucket, photo)
    indexed_faces_count = add_faces_to_collection(session, bucket, photo, collection_id)
    print("Faces indexed count: " + str(indexed_faces_count))

if __name__ == "__main__":
    main()