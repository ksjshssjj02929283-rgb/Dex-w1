import socket
import time
import threading
import subprocess
import os

# 配置参数
HOST = '0.0.0.0'  # 监听所有可用网络接口
PORT = 8080       # 端口号
BUFFER_SIZE = 1024  # 缓冲区大小

def ethercat_upload(cmd_str):
    """
    执行ethercat upload命令并返回结果字符串
    """
    try:
        # 执行ethercat命令
        cmd_list = cmd_str.split()
        result = subprocess.run(
            cmd_list,
            check=True,  # 如果命令执行失败（返回非0状态码），会抛出CalledProcessError
            stdout=subprocess.PIPE,  # 捕获标准输出
            stderr=subprocess.PIPE,  # 捕获标准错误
            text=True  # 输出直接为字符串类型（无需再decode）
        )

        # 返回命令输出结果（去除首尾空白字符）
        return result.stdout.strip()

    except subprocess.CalledProcessError as e:
        # 命令执行失败时处理
        error_msg = f"命令执行失败 (状态码: {e.returncode}): {e.stderr.strip()}"
        print(error_msg)
        return None
    except FileNotFoundError:
        # 未找到ethercat命令时处理
        print("错误: 未找到ethercat命令，请检查是否安装了EtherLab")
        return None
    except Exception as e:
        # 其他异常处理
        print(f"执行命令时发生错误: {str(e)}")
        ij
        return None

def check_service_status():
    try:
        # 使用 systemctl is-active 命令检查服务状态
        result = subprocess.run(
            ['systemctl', 'is-active', 'dexe-auto.service'],
            check=False,  # 不抛出异常，因为服务不存在或未启动时该命令会返回非零退出码
            capture_output=True,
            text=True
        )
        # 去除输出结果的空白字符并判断
        output = result.stdout.strip()
        if output == 'active':
            return 'auto_mode'
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] 查询状态时出错: {e}")

    try:
        # 使用 systemctl is-active 命令检查服务状态
        result = subprocess.run(
            ['systemctl', 'is-active', 'dexe-tele.service'],
            check=False,  # 不抛出异常，因为服务不存在或未启动时该命令会返回非零退出码
            capture_output=True,
            text=True
        )
        # 去除输出结果的空白字符并判断
        output = result.stdout.strip()
        if output == 'active':
            return 'tele_mode'
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] 查询状态时出错: {e}")

    return 'setting'

def switch_mode(mode):
    loop_time = 0
    service = mode + ".service"
    timer = mode + ".timer"

    try:
        # 关闭dexe-auto
        print("close auto")
        command = ['sudo', 'systemctl', 'stop', 'dexe-auto.service']
        result = subprocess.run(command, input='123456\n', check=True, capture_output=True, text=True, timeout=30)
        print("close auto")
        command = ['sudo', 'systemctl', 'disable', 'dexe-auto.timer']
        result = subprocess.run(command, input='123456\n', check=True, capture_output=True, text=True, timeout=30)

    except Exception as e:
        print(f"[ERROR] 重新加载 systemd 配置时出错: {e}")

    try:
        # 关闭dexe-tele
        print("close tele")
        command = ['sudo', 'systemctl', 'stop', 'dexe-tele.service']
        result = subprocess.run(command, input='123456\n', check=True, capture_output=True, text=True, timeout=30)
        print("close tele")
        command = ['sudo', 'systemctl', 'disable', 'dexe-tele.timer']
        result = subprocess.run(command, input='123456\n', check=True, capture_output=True, text=True, timeout=30)

    except Exception as e:
        print(f"[ERROR] 重新加载 systemd 配置时出错: {e}")

    try:
        # 重启服务以使更改生效
        print("open ", service)
        command = ['sudo', 'systemctl', 'enable', timer]
        result = subprocess.run(command, input='123456\n', check=True, capture_output=True, text=True, timeout=30)
        print("open ", service)
        command = ['sudo', 'systemctl', 'start', service]
        result = subprocess.run(command, input='123456\n', check=True, capture_output=True, text=True, timeout=30)
        while (check_service_status() == 'setting'):
            time.sleep(1)
            loop_time = loop_time + 1
            if (loop_time > 30):
                break

    except Exception as e:
        print(f"[ERROR] 重新加载 systemd 配置时出错: {e}")

def handle_client(client_socket, client_address):
    """处理单个客户端连接的线程函数"""
    print(f"新连接: {client_address}")

    try:
        # 发送欢迎消息
        while True:
            welcome_msg = client_socket.recv(BUFFER_SIZE).decode('utf-8')
            if welcome_msg !="":
                # print("welcome_msg: ", welcome_msg)
                str1=welcome_msg
                str2="getop"
                str3="getbat"
                str4="poweroff"
                str5="get_mode"
                str6="tele_mode"
                str7="auto_mode"
                if(str1.lower() == str2.lower()):
                    message = ethercat_upload("ethercat slaves")
                    client_socket.sendall(message.encode('utf-8'))
                elif(str1.lower() == str3.lower()):
                    message = ethercat_upload("ethercat upload --type uint8 -p 0 0x6000 3")
                    client_socket.sendall(message.encode('utf-8'))
                elif(str1.lower() == str4.lower()):
                    os.system("echo '123456' | sudo -S poweroff")
                elif(str1.lower() == str5.lower()):
                    print("welcome_msg: ", welcome_msg)
                    message = check_service_status()
                    client_socket.sendall(message.encode('utf-8'))
                elif(str1.lower() == str6.lower()):
                    print("welcome_msg: ", welcome_msg)
                    switch_mode('dexe-tele')
                    message = "finished"
                    client_socket.sendall(message.encode('utf-8'))
                elif(str1.lower() == str7.lower()):
                    print("welcome_msg: ", welcome_msg)
                    switch_mode('dexe-auto')
                    message = "finished"
                    client_socket.sendall(message.encode('utf-8'))

            time.sleep(1)
    except Exception as e:
        print(f"与 {client_address} 通信时出错: {e}")
    finally:
        # 关闭客户端连接
        client_socket.close()
        print(f"与 {client_address} 的连接已关闭")

def start_server():
    """启动TCP服务器"""
    # 创建TCP套接字 (IPv4, 流式套接字)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        # 设置端口复用，避免程序退出后端口被占用
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # 绑定地址和端口
        server_socket.bind((HOST, PORT))

        # 开始监听，最大连接数为5
        server_socket.listen(5)
        print(f"服务器已启动，监听 {HOST}:{PORT} ...")

        try:
            while True:
                # 接受客户端连接
                client_socket, client_address = server_socket.accept()

                # 创建线程处理客户端，主线程继续等待新连接
                client_thread = threading.Thread(
                    target=handle_client,
                    args=(client_socket, client_address),
                    daemon=True  # 主线程退出时自动结束子线程
                )
                client_thread.start()
                time.sleep(1)
                # print(f"当前活跃连接数: {threading.active_count() - 1}")  # 减去主线程

        except KeyboardInterrupt:
            print("\n服务器正在关闭...")
        finally:
            server_socket.close()
            print("服务器已关闭")

if __name__ == "__main__":
    start_server()