# 使用方法

## 安装依赖
```bash
# 安装sympybotics
python -m pip install --upgrade pip
git clone https://github.com/cdsousa/SymPyBotics.git
cd SymPyBotics
python3 setup.py install --user
cd .. && rm -rf SymPyBotics
# 安装pylmi_sdp
pip install -i https://mirrors.aliyun.com/pypi/simple/ pylmi_sdp
# 安装dsdp
sudo apt install dsdp
```

## 可能遇到的问题

### `lmi_sdp`与`SymPy`版本不兼容问题

如果遇到如下报错：

```
ImportError: cannot import name 'MatrixError' from 'sympy.matrices.matrices' (/home/xc/.local/lib/python3.10/site-packages/sympy/matrices/matrices.py)
```

请手动修改 `lmi_sdp` 包中的相关文件，将

```python
from sympy.matrices.matrices import MatrixError
```

改为

```python
from sympy.matrices.common import MatrixError
```

需要修改的文件包括：

- `.../python3.x/site-packages/lmi_sdp/lm.py`
- `.../python3.x/site-packages/lmi_sdp/lmi.py`