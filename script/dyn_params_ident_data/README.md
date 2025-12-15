# 使用方法

## 安装依赖
```
# 安装sympybotics
python -m pip install --upgrade pip
git clone https://github.com/cdsousa/SymPyBotics.git
cd SymPyBotics
python3 setup.py install --user
cd .. && rm -rf SymPyBotics
pip install -i https://mirrors.aliyun.com/pypi/simple/ pylmi_sdp
```

```bash
sudo apt install dsdp
```

`lmi_sdp`与`SymPy`版本不兼容问题：
ImportError: cannot import name 'MatrixError' from 'sympy.matrices.matrices' (/home/xc/.local/lib/python3.10/site-packages/sympy/matrices/matrices.py)

```bash
sudo gedit .../python3.x/site-packages/lmi_sdp/lm.py
sudo gedit .../python3.x/site-packages/lmi_sdp/lmi.py
```

from sympy.matrices.<del>matrices</del>common import MatrixError