# exit immediately on any failed step
set -xe

ARCH=$(uname -m)
echo "Detected architecture: $ARCH"

mkdir -p fcl_dependencies
cd fcl_dependencies

# git clone https://github.com/danfis/libccd.git
# git clone https://github.com/flexible-collision-library/fcl.git
# git clone https://github.com/BerkeleyAutomation/python-fcl.git
if [ ! -d octomap-1.10.0 ]; then
    wget https://github.com/OctoMap/octomap/archive/refs/tags/v1.10.0.tar.gz -O octomap.tar.gz
    tar -xzf octomap.tar.gz
fi

if [ ! -d fcl-0.7.0 ]; then
    wget https://github.com/flexible-collision-library/fcl/archive/refs/tags/0.7.0.tar.gz -O fcl.tar.gz
    tar -xzf fcl.tar.gz
fi
if [ ! -d libccd-2.1 ]; then
    wget https://github.com/danfis/libccd/archive/refs/tags/v2.1.tar.gz -O libccd.tar.gz
    tar -xzf libccd.tar.gz
fi
if [ ! -d python-fcl-0.7.0.8 ]; then
    wget https://github.com/BerkeleyAutomation/python-fcl/archive/refs/tags/v0.7.0.8.tar.gz -O python-fcl.tar.gz
    tar -xzf python-fcl.tar.gz
fi
if [ ! -d ompl ]; then
    if [ "$ARCH" == "x86_64" ]; then
        wget https://github.com/ompl/ompl/releases/download/1.7.0/wheels-ubuntu-latest-x86_64.zip -O ompl.zip
        unzip ompl.zip -d ompl
    elif [[ "$ARCH" == "aarch64" || "$ARCH" == arm* ]]; then
        wget https://github.com/ompl/ompl/releases/download/1.7.0/wheels-ubuntu-24.04-arm-aarch64.zip -O ompl.zip
        unzip ompl.zip -d ompl
    else
        echo "Unknown architecture: $ARCH"
        exit 1
    fi
fi


echo "=== Installing OMPL ==="
cd ompl
if [ "$ARCH" == "x86_64" ]; then
    pip install ompl-1.7.0-cp310-cp310-manylinux_2_28_x86_64.whl
elif [[ "$ARCH" == "aarch64" || "$ARCH" == arm* ]]; then
    pip install ompl-1.7.0-cp310-cp310-manylinux_2_28_aarch64.whl
fi


echo "=== Installing octomap ==="
cd octomap-1.10.0
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
sudo make install
cd ../..


echo "=== Installing libccd ==="

cd libccd-2.1
rm -rf build
mkdir build
cd build
cmake -G "Unix Makefiles" -DENABLE_DOUBLE_PRECISION=ON ..
make -j8
sudo make install
cd ../..

echo "=== Installing FCL ==="

cd fcl-0.7.0
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
sudo make install
cd ../..

echo "=== Updating LD_LIBRARY_PATH in ~/.bashrc ==="

echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

echo "=== Installing python-fcl ==="

cd python-fcl-0.7.0.8
pip install .