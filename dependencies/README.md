# Install Dependencies

### Common
```bash
cd dependencies
python3 -m pip install -U -r requirements.txt
```

### Ros2-humble
todo

### [rlia](http://69.235.177.182:8081/Engine/rlia)
todo

### [dexsim(optional)](http://69.235.177.182:8081/Engine/dexsim)
```bash
pip install --no-cache-dir --index-url http://192.168.3.43:8080/simple/ --trusted-host 192.168.3.43 dexsim==0.0.0
```
Notes: For some reason, please reinstall [glia](http://69.235.177.182:8081/Engine/glia) after installing dexsim


### Prebuilt whl package download
http://192.168.3.43:8080/packages/
