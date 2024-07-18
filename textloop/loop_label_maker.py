import difflib
import math
def string_similar(s1, s2): #模糊匹配法计算文本相似性
    return difflib.SequenceMatcher(None, s1, s2).quick_ratio()
#s1=['消火栓', 'Fire', 'Hydrant', '消火栓前禁止堆放杂物', '119', '12345', '天广消防股份有限公司']
#s2=['消火栓', 'Hydrant', '消火栓前禁止堆放杂物','Fire', '119', '12345', '天广消防股份有限公司']
#for i in range(len(data4_message)):
#    s1 = data4_message[i]
#   s2 = data4_answer[i]
#print(string_similar(s1, s2))

def xyz_distance(a,b): #计算欧式距离
    x1, y1, z1 = float(a[0]),float(a[1]),float(a[2])
    x2, y2, z2 = float(b[0]), float(b[1]), float(b[2])
    return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))


line_time_all=[]
line_text_all = []
line_xyz_all=[]
filename_i = 'F:\E盘SZU\IRD论文\工具代码\Text_Scene#光电楼8楼20240531 - 副本.txt'
filename_o = 'F:\E盘SZU\IRD论文\工具代码\loop_label_0.5.txt'
with open(filename_i, 'r') as file:
    #label_loop = [-1] * len(file.readlines())
    #for t in range(len(file.readlines())):
       # label_loop.append(-1)
    for line in file:
        # 使用空格分割每一行
        words = line.split()
        time=words[0]
        x=words[1]
        y=words[2]
        z=words[3]
        words.pop(0) #弹出第一个时间戳，剩下的就全是text
        words.pop(0)
        words.pop(0)
        words.pop(0)
        line_time_all.append(time)
        line_text_all.append(words)
        line_xyz_all.append([x,y,z])
#print(line_text_all)
index_i=-1
index_j=-1
loop_list=[-2]*len(line_time_all)
for i in line_text_all:
    index_i=index_i+1
    for j in line_text_all:
        index_j=index_j+1
        if index_i>=index_j:
            continue
        score=string_similar(i, j)
        distance=xyz_distance(line_xyz_all[index_i],line_xyz_all[index_j])
        if score>=0.5 and distance<=2 :#相似度大于0.7，距离小于2认为是回环
            print("loop")
            loop_list[index_i] = index_i
            loop_list[index_j] = index_i
    index_j = -1
print(loop_list)

with open(filename_o, 'w') as file:
    for time, loop in zip(line_time_all, loop_list):
        file.write(str(time)+ " " + str(loop) + '\n')
