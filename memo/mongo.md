# launch
roslaunch mongodb_store mongodb_store.launch db_path:=/home/ubuntu/aaa

# mongoを直に
https://qiita.com/saba1024/items/f2ad56f2a3ba7aaf8521

## 起動
sudo service mongodb start
sudo service mongodb stop

## データを見る
mongod --dbpath /home/ubuntu/aaa/ -rest
でサーバーを起動

mongo でウィンドウを起動
show dbs
use dbs
db.getCollectionNames()
db.message_store.find()

## db削除
use *****
db.dropDatabase()

# ROS node から書き込み

## mongoを起動
roslaunch mongodb_store mongodb_store.launch db_path:=/home/ubuntu/aaa port:=27017

# js から読み込む

## インストール
npm install mongodb --save

## requirejsのインストール
wget https://requirejs.org/docs/release/2.3.6/minified/require.js


# 参考
https://qiita.com/opengl-8080/items/196213867b859daea719
https://qiita.com/hairui/items/5e3c4de1f7e9dbf14d16