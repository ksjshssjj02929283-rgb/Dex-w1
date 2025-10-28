#!/bin/bash

# 定义规则内容
RULES=(
'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="ttyIMU"'
'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0666", SYMLINK+="ttyUSB_485"'
'SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="06a0", MODE:="0666", OWNER:="root", GROUP:="video", SYMLINK+="dabai_dcw2"'
'SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0561", MODE:="0666", OWNER:="root", GROUP:="video", SYMLINK+="dabai_dcw2_rgb"'
'SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0666", GROUP="plugdev", SYMLINK+="oculus_quest3s"'
'SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="3c:6d:66:2b:ec:fe", NAME="LAN0"'
'SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="3c:6d:66:2b:ec:f1", NAME="LAN1"'
'SUBSYSTEM=="video4linux", ATTRS{idVendor}=="1f3b", ATTRS{idProduct}=="1021", ATTR{name}=="Gesture Camera: NVCAM", ATTR{index}=="0", MODE:="0666", SYMLINK+="video99"'
)

# 定义规则文件路径
RULES_FILE="/etc/udev/rules.d/99-custom-devices.rules"

# 检查是否以root运行
if [ "$(id -u)" -ne 0 ]; then
    echo "错误：此脚本需要以root权限运行"
    exit 1
fi

# 清空或创建规则文件
echo "创建/清空UDEV规则文件 $RULES_FILE..."
> "$RULES_FILE"

# 写入所有规则
for rule in "${RULES[@]}"; do
    echo "写入规则: $rule"
    echo "$rule" >> "$RULES_FILE"
done

# 检查写入是否成功
if [ $? -eq 0 ]; then
    echo "所有规则写入成功"
else
    echo "错误：写入规则文件失败"
    exit 1
fi

# 重新加载udev规则
echo "重新加载udev规则..."
udevadm control --reload-rules && udevadm trigger

echo "UDEV规则已成功添加并重新加载"
echo "已添加以下规则："
printf '%s\n' "${RULES[@]}"