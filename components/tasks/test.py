import os

file_dir = "D:/CloudForce/CODE/ubibot-WS1-ESP/components/tasks"
for root, dirs, files in os.walk(file_dir, topdown=False):
    print(root)     # 当前目录路径
    print(dirs)     # 当前目录下所有子目录
    print(files)        # 当前路径下所有非目录子文件
