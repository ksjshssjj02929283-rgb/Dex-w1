DexEmbodied Robot Client

# 分支管理
- 机械臂通用功能在`main`和`dev`分支上进行开发
- W1人形在`w1`分支上开发，需要持续合入最新的`dev`的改动
- feat分支建议按`zyn/dev/xxx`和`fsh/w1/xxx`风格命名

# 部署
## 安装dexe_robot_client
### 初次安装
```bash
pip install -e .
```

### 添加新packages后重新安装
```bash
pip install -e . --force-reinstall
```

### 卸载
```bash
pip uninstall dexe_robot_client
```

## 编译dexe_interfaces
```bash
cd ./dexe_interfaces  
colcon build --packages-select robot_server_interfaces
```
运行python程序前在终端执行
```bash
source ./dexe_interfaces/install/setup.bash
```