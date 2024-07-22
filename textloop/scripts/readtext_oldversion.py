import os
import cv2
import pytesseract
import pandas as pd

# 配置Tesseract
#sudo apt-get update
#sudo apt-get install tesseract-ocr
#sudo apt-get install libtesseract-dev
#pip install opencv-python-headless pytesseract pandas

# 读取ggo_kf_odom.txt的xyz
#
odom_file = 'textloop/txt/ggo_kf_odom.txt'#相对路径
odom_data = pd.read_csv(odom_file, delimiter=' ', header=None, usecols=[4, 8, 12])
odom_data.columns = ['x', 'y', 'z']

# 读取图片
pictures_folder = 'textloop/fortest'
output_file = 'textloop/txt/text.txt'

# 获取图片列表
images = [f for f in os.listdir(pictures_folder) if f.endswith('.jpg')]
total_images = len(images)

# 遍历图片文件夹中的所有图片
for idx, filename in enumerate(images, 1):
    timestamp = filename.split('.')[0]  # 提取时间戳
    img_path = os.path.join(pictures_folder, filename)
    img = cv2.imread(img_path)

    # 使用Pytesseract进行文本识别
    text = pytesseract.image_to_string(img)

    # 如果图片中有文本信息
    if text.strip():
        # 获取对应时间戳的坐标数据
        index = int(timestamp)  # 确保时间戳可以转换为整数并用作索引
        if index < len(odom_data):
            x, y, z = odom_data.loc[index]

            # 写入输出文件
            with open(output_file, 'a') as out_file:
                out_file.write(f'{timestamp} {x} {y} {z} {text.strip()}\n')

    # 输出当前处理进度
    print(f'Processing image {idx} of {total_images}: {filename}')

print("文本识别和数据写入完成。")