#!/usr/bin/env python
# -*- coding: utf-8 -*-

import requests
from bs4 import BeautifulSoup

def check_iw(target_url):
    result=False
    r = requests.get(target_url)
    soup = BeautifulSoup(r.text, "html.parser")
    for a in soup.find_all('input'):
        if(a.get('alt')==u'カートに入れる'):
            result=True
    return result

'''
item_list=[
    u"ローズゴブランフレアージャンパースカート",
    u"バイオリン刺繍フレアージャンパースカート",
    u"奇跡の夜空バッスルジャンパースカート",
    u"奇跡の夜空聖歌隊ジャンパースカート ",
    u"奇跡の夜空スカート",
    u"リバイバルナポレオンワンピース",
]
'''

url_list=[
    u'http://innocent-w.jp/fs/innocentworld/jumperskirt/173705',
    u'http://innocent-w.jp/fs/innocentworld/jumperskirt/163717',
    u'http://innocent-w.jp/fs/innocentworld/jumperskirt/154719',
    u'http://innocent-w.jp/fs/innocentworld/jumperskirt/154720',
    u'http://innocent-w.jp/fs/innocentworld/bottom/153204',
    u'http://innocent-w.jp/fs/innocentworld/one-piece/181602'
]

result0=check_iw(url_list[0])
result1=check_iw(url_list[1])
if(result0 and not result1):
    print("Connect OK")
else:
    print("Connect NG")

for i in range(2,len(url_list)):
    print(check_iw(url_list[i]))
    
        



