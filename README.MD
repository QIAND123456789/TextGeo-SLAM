# TextGeo_SLAM
## 1.REQUIREMENTS
***lv_slam*** requires the following libraries:
- OpenMP
- PCL
- g2o
- suitesparse
- Sophus
- OpenCV 
- DBoW3
- A-LOAM
***lv_slam*** requires the following dependencies:
1. Anaconda & Paddle
https://mirrors.tuna.tsinghua.edu.cn/anaconda/archive/?C=M&O=D

Python 3.8.19

```
conda create -n ocr
conda activate ocr  
```

change the source:

```
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free/
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main/
conda config --set show_channel_urls yes
```
for CPU Paddleocr:
```
conda install paddlepaddle==3.0.0b1 -c paddle
```

for GPU Paddleocr:

-CUDA 11.8
```
conda install paddlepaddle-gpu==3.0.0b1 paddlepaddle-cuda=11.8 -c paddle -c nvidia
```

-CUDA 12.3
```
conda install paddlepaddle-gpu==3.0.0b1 paddlepaddle-cuda=12.3 -c paddle -c nvidia
```

Test for installation successfully:
```
python3
import paddle
paddle.utils.run_check()
```
If the ouput prints "PaddlePaddle is installed successfully!", it shows Paddle is successfully installed in the env.

2. Paddleocr bag installation
```
pip install paddleocr
```

3. Other dependencies
terminal:
cd textloop/

```
pip install -r requirements.txt 
```


## Step1-textloop

## textloop(run follow the order)
### outputimage_frombag.py 
// 从bag中提取imgs
### paddleocr.py
// imgs处理，提取文本到textloop/txt/Text_Scene-光电楼8楼20240531.txt（还没改）
// 数据结构 time，‘TEXT’
### test copy.py
// 数据结构处理， time，‘TEXT’label
### read_text.py
// 得到text.txt,包括 时间戳，x,y,z,‘TEXT’信息,保存到textloop/txt/text.txt，供launch使用



# Step2-slamspace
cd slamspace/lv_slam/

## first terminal 
```
source devel/setup.bash 
roslaunch lv_slam dlo_lfa_ggo_szu.launch calib_file:='/home/jq/jq/GML-WORKSPACE/chun/slamspace/lv_slam/src/lv_slam/config/szu_calib/calib.txt'     odom_file:='/home/jq/jq/GML-WORKSPACE/chun/slamspace/lv_slam/src/lv_slam/data/szu_lv_dlo_lfa_ggo/dlo_lfa_global/data/szu_01_odom.txt' seq:=01  lfa_output_path:='/home/jq/jq/GML-WORKSPACE/chun/slamspace/lv_slam/src/lv_slam/data/szu_lv_dlo_lfa_ggo' label_path:="/media/chenshoubin/HD_CHEN_4T/02data/szu_data/726data/new2_2021-07-31-17-06-25_label.txt"
```
注意：第一遍不改label_path,第二遍改成textloop/text.text的绝对路径
## 2nd terminal
```
rosbag play --clock  xx.bag rslidar_points:=/velodyne_points   -r 0.5
```
OR
```
rosbag play --clock 2024-07-03-21-44-03.bag  /rslidar_points:=/velodyne_points    -r 0.5
```
## 3rd terminal
```
source devel/setup.bash 
rosservice call /global_graph/dump "destination: '/home/jq/jq/GML-WORKSPACE/chun/slamspace/lv_slam/src/lv_slam/data/szu_lv_dlo_lfa_ggo/data/'  "

rosservice call /global_graph/save_map "{resolution: 0.05, destination: '/home/jq/jq/GML-WORKSPACE/chun/slamspace/lv_slam/src/lv_slam/data/szu_lv_dlo_lfa_ggo/data/map.pcd'}"
```


