**git快速入门手册**

##### 一、本地上传

```
git config --global http.version HTTP/1.1
git config --global http.proxy http://127.0.0.1:7897
git config --global https.proxy http://127.0.0.1:7897
# 把 Git 的代理端口设置成 7897
```

```
git init

echo .idea/ >> .gitignore
echo */.idea/ >> .gitignore

git rm -r --cached .idea
git rm -r --cached picture_solve/.idea

git add .
git commit -m "upload files"

git branch -M main

git remote add origin https://github.com/你的用户名/jianza.git

git push -u origin main
```

##### 二、本地更新上传

```
git status
# 查看更新
cd C:\Users\27442\Desktop\jianza\ultralytics-main
# 打开文件夹
git add .
git commit -m "update code"
git push
```

##### 三、云端上传本地

```
# 本地已经存在项目情况
cd C:\Users\27442\Desktop\jianza\ultralytics-main
git pull
# 本地没有存在项目情况
git clone https://github.com/kakaximo123/YOLOv11-OBB-V1.git
# 判断云端是否存在更新
cd C:\Users\27442\Desktop\jianza\ultralytics-main
git fetch
git status
# 查看云端比本地多哪些提交
git log HEAD..origin/main --oneline
```

