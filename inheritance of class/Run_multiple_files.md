
[toc]

<font face='宋体'>

# 如何在vscode中使用cmake

## 一、编写CMakeLists.txt文件
### 1. 核心逻辑
- 最低版本要求：指定支持的 CMake 最低版本（避免兼容性问题）。
- 项目信息：定义项目名称、支持的语言（C/C++）。
- 编译配置：设置 C++ 标准、编译选项（警告、优化等）。
- 源文件管理：指定参与编译的 .cpp 源文件和 .hpp 头文件。
- 目标产物：生成可执行文件或库文件（静态 / 动态库）。
- 依赖管理：（可选）链接第三方库、设置头文件搜索路径。  
        
### 2. 通用语法与关键指令
|指令|作用说明	|示例|
|:---|:--|:---|
|`cmake_minimum_required(VERSION x.y)`	|指定最低 CMake 版本（建议 ≥3.10）|	`cmake_minimum_required(VERSION 3.10)`|
|`project(ProjectName [LANGUAGES CXX])`|	定义项目名称，指定语言（默认 C/C++）	|`project(BigintCalculator LANGUAGES CXX)`|
`set(VAR VALUE)`|	定义变量（如 C++ 标准、源文件列表）	|`set(CMAKE_CXX_STANDARD 11)`|
`add_executable(TargetName ${SOURCES})`|	生成可执行文件，指定目标名和源文件|	`add_executable(bigint_calc ${SOURCES})`|
`target_include_directories(TargetName PUBLIC 路径)`|	设置头文件搜索路径（第三方库时常用）|	`target_include_directories(bigint_calc PUBLIC ./include)`|
`target_compile_options(TargetName PRIVATE 选项)`	|添加编译选项（如警告等级、优化级别）|	`target_compile_options(bigint_calc PRIVATE -Wall -O2)`|

- example:
```cmake
# 1. 最低 CMake 版本要求（根据你的 CMake 版本调整，建议 ≥3.10）
cmake_minimum_required(VERSION 3.10)

# 2. 项目配置：名称 + 支持 C++ 语言
project(BigintCalculator LANGUAGES CXX)

# 3. 编译规则配置
# 设置 C++ 标准（必须 ≥C++11，匹配你的代码语法）
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # 强制要求该 C++ 标准，否则报错

# （可选）添加编译选项：开启警告、优化级别
if(MSVC)
  # Windows Visual Studio 编译器：开启所有警告
  target_compile_options(bigint_calc PRIVATE /W4)
else()
  # Linux/macOS/GCC/Clang：开启警告 + O2 优化
  target_compile_options(bigint_calc PRIVATE -Wall -Wextra -O2)
endif()

# 4. 源文件列表（明确指定所有 .cpp 文件，避免遗漏）
set(SOURCES
  bigint.cpp
  main.cpp
)

# （可选）头文件列表（仅用于 IDE 索引，不影响编译，但建议添加）
set(HEADERS
  bigint.hpp
)

# 5. 生成可执行文件：目标名（bigint_calc）+ 源文件
add_executable(bigint_calc ${SOURCES} ${HEADERS})

# （可选）如果头文件在其他目录（如 ./include），添加搜索路径
# target_include_directories(bigint_calc PUBLIC ./include)
```

### 3. 注意事项
- 路径问题：
   
   - 源文件路径相对于 CMakeLists.txt 的位置（同目录直接写文件名，子目  录用 ./subdir/file.cpp）
   - 避免中文路径和特殊字符（如空格、括号），否则可能导致编译失败
- 多文件管理：
   - 源文件较多时，可使用 file(GLOB SOURCES "src/*.cpp") 批量获取，但不建议（可能遗漏文件），推荐显式列出
  
## 二、运行多文件

### 1. CMake的配置
- 按下 `Ctrl+Shift+P`（Windows/Linux）或 `Cmd+Shift+P`（macOS），打开命令面板。
- 输入 `CMake: 配置`，选择编译器（根据你的环境选择）：
  
   - Windows：选择 `Visual Studio 17 2022`（若装了 MinGW，选 MinGW Makefiles），或者`Visual Studio Community 2022 Release - amd64`。
   - Linux：选择 `GCC`。
   - macOS：选择 `Clang`。
- 配置成功后，项目根目录会生成 `build` 文件夹（存放编译产物）。
### 2. CMake的构建
- 方式 1：点击`VS Code`底部状态栏的 `Build` 按钮（锤子图标）。
- 方式 2：命令面板输入 `CMake: 构建`。
- 构建成功提示：`Build finished with exit code 0`，可执行文件生成在 `build/Debug/*.exe（Windows）`或 `build/*_calc（Linux/macOS）`。
### 3. 运行文件
- 按`Ctrl+Shift+p`，打开命令面板。
- 输入 `CMake: Run Without Debugging`（或按`Ctrl+F5`），运行程序。