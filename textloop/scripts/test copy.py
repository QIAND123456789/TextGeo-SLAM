import difflib
import math

def string_similar(s1, s2): # 模糊匹配法计算文本相似性
    return difflib.SequenceMatcher(None, s1, s2).quick_ratio()

def xyz_distance(a, b): # 计算欧式距离
    x1, y1, z1 = float(a[0]), float(a[1]), float(a[2])
    x2, y2, z2 = float(b[0]), float(b[1]), float(b[2])
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

line_time_all = []
line_text_all = []
line_xyz_all = []

filename_i = "textloop/txt/Text_Scene-光电楼8楼20240531.txt"
filename_o = 'textloop/txt/loop_label_0.5.txt'

with open(filename_i, 'r', encoding='latin1') as file:  # 尝试使用 'latin1' 编码
    for line in file:
        words = line.split()
        time = words[0]
        x, y, z = words[1], words[2], words[3]
        line_time_all.append(time)
        line_text_all.append(words[4:]) # 从第五个元素开始为文本
        line_xyz_all.append([x, y, z])

loop_list = [-2] * len(line_time_all)

for index_i, i in enumerate(line_text_all):
    for index_j in range(index_i + 1, len(line_text_all)):
        j = line_text_all[index_j]
        score = string_similar(' '.join(i), ' '.join(j))
        distance = xyz_distance(line_xyz_all[index_i], line_xyz_all[index_j])
        if score >= 0.5 and distance <= 2: # 相似度大于0.5，距离小于2认为是回环
            loop_list[index_i] = index_i
            loop_list[index_j] = index_i

with open(filename_o, 'w') as file:
    for time, loop in zip(line_time_all, loop_list):
        file.write(f"{time} {loop}\n")
