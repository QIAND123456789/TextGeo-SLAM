import os
import cv2
import pytesseract
import pandas as pd
from PIL import Image
from concurrent.futures import ThreadPoolExecutor, as_completed

# 配置Tesseract
# sudo apt-get update
# sudo apt-get install tesseract-ocr
# sudo apt-get install tesseract-ocr-chi-sim
# sudo apt-get install tesseract-ocr-chi-tra
# pip install opencv-python-headless pytesseract pandas

# 读取ggo_kf_odom.txt的xyz
odom_file = 'textloop/txt/ggo_kf_odom.txt'  # 相对路径
odom_data = pd.read_csv(odom_file, delimiter=' ', header=None, usecols=[4, 8, 12])
odom_data.columns = ['x', 'y', 'z']

# 读取图片
pictures_folder = 'textloop/pictures'
output_file = 'textloop/txt/text.txt'

# 获取图片列表
images = [f for f in os.listdir(pictures_folder) if f.endswith('.jpg')]
total_images = len(images)

# 创建输出文件，写入标题行
with open(output_file, 'w') as out_file:
    out_file.write('timestamp x y z text\n')

def process_image(filename):
    timestamp = filename.split('.')[0]  # 提取时间戳
    img_path = os.path.join(pictures_folder, filename)
    img = cv2.imread(img_path)
    if img is None:
        print(f'Failed to read image: {img_path}')
        return None

    print(f'Processing image: {img_path}')  # 调试信息

    # 将图片转换为PIL格式并设置DPI
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(img_rgb)
    pil_img.info['dpi'] = (300, 300)  # 设置一个合理的DPI

    # 使用Pytesseract进行文本识别
    try:
        text = pytesseract.image_to_string(pil_img, lang='chi_sim')
    except Exception as e:
        print(f'Error in text recognition for {filename}: {e}')
        return None

    print(f'Text for {filename}: {text.strip()}')  # 调试信息

    # 如果图片中有文本信息
    if text.strip():
        # 获取对应时间戳的坐标数据
        try:
            index = int(timestamp)  # 确保时间戳可以转换为整数并用作索引
            if index < len(odom_data):
                x, y, z = odom_data.loc[index]
                # 返回结果
                return f'{timestamp} {x} {y} {z} {text.strip()}'
        except Exception as e:
            print(f'Error in processing odom data for {filename}: {e}')
    return None

# 使用ThreadPoolExecutor进行并行处理，并管理进度
with ThreadPoolExecutor() as executor:
    future_to_filename = {executor.submit(process_image, img): img for img in images}
    for idx, future in enumerate(as_completed(future_to_filename), 1):
        result = future.result()
        if result:
            with open(output_file, 'a') as out_file:
                out_file.write(result + '\n')
            print(f'Written result: {result}')  # 调试信息
        print(f'Processing image {idx} of {total_images}')

print("文本识别和数据写入完成。")
