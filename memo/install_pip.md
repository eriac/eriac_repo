# 注意
aptで入れるとver8が入りver8 ver9の間は互換性がないために、アップグレードに失敗する。aptでは入れずにpythonを使って入れよう。

# コマンド

```
sudo apt-get remove python-pip
curl "https://bootstrap.pypa.io/get-pip.py" -o "get-pip.py"
sudo python get-pip.py
```