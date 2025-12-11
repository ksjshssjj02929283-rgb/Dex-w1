# 使用方法
## 文件结构
---
- data/trajectories 生成的参考轨迹 (q)
- data/recdata 执行参考轨迹得到的关节状态变化 (t, q, tau)
- data/robotparams 机器人CAD参数，用于和最终辨识结果比较
---
- tmp/solutions 辨识结果

## 安装依赖
Python
- SymPy
- SymPyBotics
- PyLMI-SDP

git clone https://github.com/cdsousa/SymPyBotics.git
cd SymPyBotics
python3 setup.py install --user
pip install ipykernel pylmi_sdp -i https://mirrors.aliyun.com/pypi/simple/
python3 -m pip install --user --upgrade pip setuptools wheel matplotlib ipython ipykernel matplotlib-inline -i https://mirrors.aliyun.com/pypi/simple/

```bash
sudo apt install dsdp

# 2to3 -w support_funcs/
```

`lmi_sdp`与`SymPy`版本不兼容问题：
ImportError: cannot import name 'MatrixError' from 'sympy.matrices.matrices' (/home/xc/.local/lib/python3.10/site-packages/sympy/matrices/matrices.py)

```bash
sudo gedit .../python3.x/site-packages/lmi_sdp/lm.py
sudo gedit .../python3.x/site-packages/lmi_sdp/lmi.py
```

from sympy.matrices.<del>matrices</del>common import MatrixError