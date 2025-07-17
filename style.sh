set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${YELLOW}开始格式化 C++ 代码...${NC}"

if ! command -v clang-format &> /dev/null; then
    echo -e "${RED}错误: clang-format 未安装${NC}"
    echo "请运行: sudo apt install clang-format"
    exit 1
fi

# 定义格式化配置
STYLE_CONFIG="{BasedOnStyle: Google, IndentWidth: 4, BreakBeforeBraces: Allman, AllowShortIfStatementsOnASingleLine: false, IndentCaseLabels: false, ColumnLimit: 100}"

# 格式化所有 C++ 文件，排除编译目录
echo -e "${BLUE}正在格式化文件...${NC}"
find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.cc" -o -name "*.cxx" \) \
    -not -path "./build/*" \
    -not -path "./install/*" \
    -not -path "./log/*" \
    -not -path "./.git/*" \
    -exec clang-format -i --style="$STYLE_CONFIG" {} \;

echo -e "${GREEN}代码格式化完成!${NC}"

# 可选：显示格式化的文件数量
file_count=$(find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.cc" -o -name "*.cxx" \) \
    -not -path "./build/*" \
    -not -path "./install/*" \
    -not -path "./log/*" \
    -not -path "./.git/*" | wc -l)

echo -e "${GREEN}共处理了 $file_count 个文件${NC}"