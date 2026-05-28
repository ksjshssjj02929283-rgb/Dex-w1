#!/bin/bash
set -e

# 从 w1_server/CMakeLists.txt 中读取版本号
PKG_VERSION=$(grep 'set(W1_SERVER_VERSION' w1_server/CMakeLists.txt | sed -e 's/set(W1_SERVER_VERSION "\(.*\)")/\1/')

PKG_NAME="dexe-robot-server"
PKG_ARCH="arm64"
PKG_DIR="deb_build"
GIT_HASH=$(git rev-parse --short HEAD)
GIT_DATE=$(git log -1 --format=%cd --date=format:'%Y%m%d')
LIB_INSTALL_PREFIX="/home/dexforce/w1/install"
SCRIPTS_INSTALL_PREFIX="/home/dexforce/w1/dexe_robot_server"

echo "Step 1: Building project (Version: $PKG_VERSION)..."
./build.sh

echo "Step 2: Preparing DEB structure..."
rm -rf ${PKG_DIR}
mkdir -p ${PKG_DIR}${LIB_INSTALL_PREFIX}
mkdir -p ${PKG_DIR}${SCRIPTS_INSTALL_PREFIX}
mkdir -p ${PKG_DIR}/DEBIAN
mkdir -p ${PKG_DIR}/etc/ld.so.conf.d

if [ -d "../install" ]; then
    cp -r ../install/* ${PKG_DIR}${LIB_INSTALL_PREFIX}/
else
    echo "❌ Error: ../install directory not found."
    exit 1
fi

echo "Step 2.1: Copying scripts..."
if [ -d "script" ]; then
    cp -r script ${PKG_DIR}${SCRIPTS_INSTALL_PREFIX}/
fi
cp *.sh ${PKG_DIR}${SCRIPTS_INSTALL_PREFIX}/

echo "Step 2.2: Pruning non-runtime files for smaller package..."

# 删除 Python 缓存，不影响运行。
find "${PKG_DIR}" -type d -name '__pycache__' -prune -exec rm -rf {} +
find "${PKG_DIR}" -type f -name '*.pyc' -delete

# 删除静态库/调试残留，运行时动态链接不依赖这些文件。
find "${PKG_DIR}${LIB_INSTALL_PREFIX}" -type f \( -name '*.a' -o -name '*.la' -o -name '*.pdb' \) -delete

# 删除开发期头文件、CMake 包配置和 pkgconfig，仅运行环境不需要。
find "${PKG_DIR}${LIB_INSTALL_PREFIX}" -type d -path '*/include' -prune -exec rm -rf {} +
find "${PKG_DIR}${LIB_INSTALL_PREFIX}" -type d -path '*/share/*/cmake' -prune -exec rm -rf {} +
find "${PKG_DIR}${LIB_INSTALL_PREFIX}" -type d \( -path '*/lib/pkgconfig' -o -path '*/share/pkgconfig' \) -prune -exec rm -rf {} +

echo "Step 3: Creating control file..."
cat <<INFO > ${PKG_DIR}/DEBIAN/control
Package: ${PKG_NAME}
Version: ${PKG_VERSION}
Section: misc
Priority: optional
Architecture: ${PKG_ARCH}
Maintainer: GitHub Copilot
Depends: ros-humble-desktop, libyaml-cpp-dev
Description: Dexe Robot Server for W1
INFO

echo "Step 4: Creating postinst script..."
cat <<POST > ${PKG_DIR}/DEBIAN/postinst
#!/bin/bash
set -e
BINS=(
    "${LIB_INSTALL_PREFIX}/w1_server/lib/w1_server/w1_server"
    "${LIB_INSTALL_PREFIX}/w1_server/lib/w1_server/feedback"
    "${LIB_INSTALL_PREFIX}/w1_server/lib/w1_server/joint_control"
)
for BIN in "\${BINS[@]}"; do
    if [ -f "\$BIN" ]; then
        setcap 'cap_sys_nice,cap_ipc_lock=+ep' "\$BIN" || true
    fi
done
cat <<LD > /etc/ld.so.conf.d/w1_server.conf
/opt/ros/humble/lib
/opt/ros/humble/lib/aarch64-linux-gnu
${LIB_INSTALL_PREFIX}/chassis_interfaces/lib
${LIB_INSTALL_PREFIX}/joint_interfaces/lib
${LIB_INSTALL_PREFIX}/robot_server_interfaces/lib
${LIB_INSTALL_PREFIX}/robot_server_lib/lib
LD
ldconfig
exit 0
POST
chmod 755 ${PKG_DIR}/DEBIAN/postinst

echo "Step 5: Building DEB package..."
DEB_FILE="${PKG_NAME}-${PKG_VERSION}-${GIT_DATE}-${GIT_HASH}-${PKG_ARCH}.deb"
dpkg-deb --build ${PKG_DIR} ${DEB_FILE}
echo "✅ Success! Package created: ${DEB_FILE}"
