# 1. 从运行参数传入文件夹路径
# 2. 列出文件夹中所有的json文件
# 3. 对列表进行打乱
# 4. 对每一个json文件：
#    a. python slowly_move_to.py <json_file> 100
#    b. python replay.py <json_file>
# 5. 回到3，并循环

import os
import random
import subprocess
import sys

def shuffle_replay_in_folder(folder_path):
    # 获取文件夹中所有的json文件
    json_files = [f for f in os.listdir(folder_path) if f.endswith('.json')]

    if not json_files:
        print("没有找到任何json文件。")
        return

    # 打乱文件列表
    random.shuffle(json_files)

    for json_file in json_files:
        json_path = os.path.join(folder_path, json_file)

        # 执行slowly_move_to.py脚本
        subprocess.run([sys.executable, 'slowly_move_to.py', json_path, '100'])

        # 执行replay.py脚本
        subprocess.run([sys.executable, 'replay.py', json_path])

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("用法: python shuffle_replay_in_folder.py <folder_path>")
        sys.exit(1)

    folder_path = sys.argv[1]

    if not os.path.isdir(folder_path):
        print(f"错误: {folder_path} 不是一个有效的文件夹路径。")
        sys.exit(1)

    while True:
        shuffle_replay_in_folder(folder_path)