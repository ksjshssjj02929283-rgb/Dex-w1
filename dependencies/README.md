# Install Dependencies

### DexeRP >= V0.1.0
Downloads from http://192.168.3.120/dexe_release/dexe_robot_planning/ and `pip install`

### OMPL & FCL
```bash
cd dependencies
bash ./install_planner_dependencies.sh
```

### Ros2-humble

### [rlia(optional)](http://69.235.177.182:8081/Engine/rlia)
```bash
pip install --no-cache-dir --index-url http://192.168.3.43:8080/simple/ --trusted-host 192.168.3.43 rlia==0.0.0
```

### [dexsim(optional)](http://69.235.177.182:8081/Engine/dexsim)
```bash
pip install --no-cache-dir --index-url http://192.168.3.43:8080/simple/ --trusted-host 192.168.3.43 dexsim==0.0.0
```

### Prebuilt whl package download
http://192.168.3.43:8080/packages/
