
<center><font color=pink face='楷体' size=7> 做字符串相关题中遇到的fouctions</font></center>            
<font face='宋体'>

[toc]

# character-string

## 1. push_back
> ### functions :
- 向容器尾部追加元素，时间复杂度通常为 O(1)
- 会创建新元素的副本（或移动副本，C++11 后支持右值引用），原元素不会被修改
- 对于 std::vector，若当前容量不足以容纳新元素，会自动分配更大的内存空间，并将原有元素拷贝 / 移动到新空间
> ### range of application :
- vector、queue、list、string
> ### example :
```c++
#include <vector>
#include <iostream>

int main() {
    std::vector<int> vec;  // 空向量
    int num = 10;
    
    vec.push_back(num);    // 向尾部添加 num 的拷贝（左值）
    vec.push_back(20);     // 向尾部添加字面量 20（隐式构造临时对象）
    
    // 输出：10 20
    for (int x : vec) {
        std::cout << x << " ";
    }
    return 0;
}
```

## 2. getline :
> ### usage forms:
- 全局函数 `std::getline()`（定义在 <string> 头文件）：读取流（如 cin、文件流）中的一整行到 `std::string`,不用担心 “输入超长” 的问题
> - grammar:getline(输入流, 接收字符串变量, 终止符)
> - 终止符（可选）：默认是 '\n'（换行符），遇到该字符停止读取 **（终止符不会存入字符串）**
> - 解决 cin >> 无法读取带空格的字符串的问题
```c++
#include <string>
#include <iostream>
using namespace std;

int main() {
    string str;
    getline(cin, str);  // 输入 1000 个字符也没问题，string 自动扩容
    cout << "长度：" << str.size() << endl;  // 正确输出实际长度
    return 0;
}
```
- 类成员函数 `istream::getline()（`定义在 <iostream> 头文件）：读取流到字符数组（char[]）
> - grammar:输入流.getline(字符数组, 数组大小, 终止符)
> - 终止符（可选）：默认 '\n'，读取到后停止，且会在字符数组末尾自动添加 '\0'（字符串结束符）

```c++
#include <iostream>
using namespace std;

int main() {
    char buf[5];  // 最多存 4 个有效字符（第 5 个是 '\0'）
    cin.getline(buf, 5);  // 输入 "12345"（5 个字符）
    
    cout << buf << endl;  // 输出 "1234"（截断超出部分）
    cout << cin.fail() << endl;  // 输出 1（流错误状态）
    
    // 后续读取失效，需重置
    cin.clear();  // 重置流状态
    return 0;
}
```

> ### matters:
-  如果先使用 cin >> 读取数据（如整数、单个单词），再用 `getline()`，会发现 `getline() ``直接读取到空字符串 —— 因为 cin >>` 会留下换行符在输入缓冲区，`getline() `会把这个换行符当作 “空行” 读取。
>> solutions :
>> 1. 用 `cin.ignore() `清空缓冲区的换行符         
>> - `cin.ignore(n, ch) `会忽略输入缓冲区中前 n 个字符，直到遇到 ch（默认忽略 1 个字符）：
```c++
cin >> age;
cin.ignore();  // 忽略缓冲区中残留的 '\n'（默认忽略 1 个字符）
getline(cin, name);  // 正常读取
```
>> - 若缓冲区可能有多个残留字符（如输入年龄后输入了多余空格），可忽略所有字符直到换行：
```c++
cin.ignore(numeric_limits<streamsize>::max(), '\n');  // 需要包含 <limits> 头文件
```
>> 2. 用 getline() 读取前先清空缓冲区（兼容多残留字符）
>> - cin >> age;
// 清空缓冲区中所有字符，直到换行
```c++
while (cin.get() != '\n');  
getline(cin, name);
```
## 3. substr ：
### grammar : 
` std::string substr(size_t pos = 0, size_t count = npos) `
- explanation:
> 1. pos（可选）：子串的起始位置索引（从 0 开始，默认值为 0，即从字符串开头截取）
> 2. count（可选）：要截取的字符个数（默认值为 std::string::npos，表示截取从 pos 到字符串末尾的所有字符）
### caution :
- 若 pos 等于字符串的长度`（str.size()）`：返回空字符串（无错误）
- 若 pos 大于字符串的长度：抛出 `std::out_of_range` 异常（程序崩溃，需避免）
- 若 pos + count 超过字符串的长度，不会报错，而是截取到字符串末尾（自动缩短 count）
- substr 返回的是 新字符串，无论怎么截取，原字符串的内容和长度都不变
### example :        
```c++
string str = "abcdefgh";
size_t n = 3;

// 前 N 个字符
string prefix = str.substr(0, n);  // abc
// 后 N 个字符（先计算起始位置：总长度 - N）
string suffix = str.substr(str.size() - n);  // fgh
cout << "前缀：" << prefix << "，后缀：" << suffix << endl;
```
## 4. reverse :
### header file : 
`<algorithm>`
### grammar : 
`std::reverse(迭代器 first, 迭代器 last)`
- key description :
> 1. first：反转的起始迭代器（指向要反转的第一个元素）
> 2. last：反转的结束迭代器（指向要反转的最后一个元素的**下一个位置**，即 **“左闭右开”**）
> 3. 操作结果：直接修改原容器 / 数组（原地反转，无额外内存开销），时间复杂度 O (n)（n 为区间内元素个数）
- caution :
> - 普通数组没有 `begin()/end()` 成员函数，需用 数组首地址（arr）和 尾后地址（arr + 长度）作为迭代器 :
```c++
int arr[] = {10, 20, 30};
int len = 3;
reverse(arr, arr + len);  // 正确：arr 是首指针，arr+3 是尾后指针
// 错误写法：reverse(arr, arr + len - 1); → 只会反转前 2 个元素（10,20→20,10）
```
> - 与 std::string::rbegin () 的区别 :
string 的 rbegin()（反向迭代器）是 “遍历反转”，不会修改原字符串；而 reverse 是 “修改原字符串反转”     
```c++
string str = "123";
// 用 rbegin() 遍历（原字符串不变）
for (auto it = str.rbegin(); it != str.rend(); ++it) {
    cout << *it;  // 输出：321
}
cout << "，原字符串：" << str << endl;  // 输出：123（未修改）

reverse(str.begin(), str.end());  // 修改原字符串
cout << "反转后原字符串：" << str << endl;  // 输出：321
```
## 5. find :
### header file :
 `<algorithm>`
### 用于字符串查找 ：
#### grammar :
```c++
#include <string>  // 必须包含头文件

// 查找子串 s，从 pos 位置开始（默认 pos=0）
size_t find(const string& s, size_t pos = 0) const;

// 查找单个字符 c，从 pos 位置开始（默认 pos=0）
size_t find(char c, size_t pos = 0) const;

// 查找 C 风格字符串（const char*），从 pos 位置开始
size_t find(const char* s, size_t pos = 0) const;
```
- 返回值 ：
&#9675; 找到：返回目标的 起始索引（`size_t` 类型，非负整数）
&#9675; 未找到：返回 `std::string::npos`（一个静态常量，表示 “无此位置”）
&#9679; `std::string::npos` 本质是 `size_t` 类型的最大值（通常是 4294967295），判断时需用` ==` 而非 `<0`（`size_t` 是无符号类型，不会小于 0）
#### Derivative functions:
- 除了基础 `find()，string `还提供了其他查找函数，覆盖不同场景 ：
  
> |函数名|	功能|示例（基于 str = "abacada"）|
> |:---|:----|:---|
> |rfind(s, pos)|从 pos 开始 反向查找（从后往前找），返回最后一次出现的位置|	str.rfind('a') → 6（最后一个 'a'）|
> |find_first_of(s)|查找 s 中 任意一个字符 首次出现的位置|str.find_first_of("cd") → 2（'c' 首次出现）|
> |find_last_of(s)|查找 s 中 任意一个字符 最后出现的位置|str.find_last_of("cd") → 4（'d' 最后出现）|
> |find_first_not_of(s)|查找不在 s 中 的第一个字符位置|str.find_first_not_of("ab") → 2（'c' 不在 "ab" 中）|

### 用于容器/数组查找 :
#### grammar : 
```c++
#include <algorithm>  // 必须包含头文件

// 查找 [first, last) 区间内第一个等于 value 的元素
template <class InputIterator, class T>
InputIterator find(InputIterator first, InputIterator last, const T& value);
```
- parameter specification :
&#9675; `first/last`：查找的 “左闭右开” 区间（迭代器 / 指针）
&#9675; `value`：要查找的目标元素（必须与容器元素类型一致）
- returned value :
&#9675; 找到：返回指向目标元素的 迭代器（或数组指针）
&#9675; 未找到：返回 last（区间的尾后迭代器 / 指针）

&#9679; example 1 (`vector`):
```c++
#include <algorithm>
#include <vector>
#include <iostream>
using namespace std;

int main() {
    vector<int> vec = {10, 20, 30, 40, 50};
    int target = 30;

    // 查找 vec 中第一个等于 30 的元素（区间：整个 vector）
    auto it = find(vec.begin(), vec.end(), target);
    if (it != vec.end()) {  // 找到：迭代器不等于尾后迭代器
        // 输出元素值和索引（索引 = 迭代器 - 起始迭代器）
        cout << "找到 " << target << "，值：" << *it << "，索引：" << it - vec.begin() << endl;
        // 输出：找到 30，值：30，索引：2
    } else {
        cout << "未找到 " << target << endl;
    }

    return 0;
}
```
&#9679; example 2 (`pointer`) :
```c++
#include <algorithm>
#include <iostream>
using namespace std;

int main() {
    int arr[] = {5, 15, 25, 35};
    int len = sizeof(arr) / sizeof(arr[0]);  // 数组长度 4
    int target = 25;

    // 查找数组中第一个等于 25 的元素（区间：arr[0] ~ arr[3]）
    int* ptr = find(arr, arr + len, target);
    if (ptr != arr + len) {  // 找到：指针不等于尾后指针
        cout << "找到 " << target << "，值：" << *ptr << "，索引：" << ptr - arr << endl;
        // 输出：找到 25，值：25，索引：2
    }

    return 0;
}
```
### 未找到的判断方式（核心避坑）:

- `string::find()`：必须用 `==` `string::npos` 判断，不能用 <0（`size_t` 是无符号类型，返回值永远非负）：
```
// 错误写法（永远为 false）
if (str.find("target") < 0) { ... }
// 正确写法
if (str.find("target") == string::npos) { ... }
```
- `std::find()`（全局函数）：用返回值 `==` `last` 判断（迭代器 / 指针等于尾后标识）：
```c++
// vector 示例
auto it = find(vec.begin(), vec.end(), target);
if (it == vec.end()) { /* 未找到 */ }

// 数组示例
int* ptr = find(arr, arr + len, target);
if (ptr == arr + len) { /* 未找到 */ }
```

## 6. strtok :
### 函数原型：
-  标准原型（线程不安全版）
```c++
char *strtok(char *str, const char *delimiters);
```
- 可重入版（线程安全，推荐使用）
```c++
char *strtok_r(char *str, const char *delimiters, char **saveptr);
```
- 参数说明：
    - `str`：第一次调用时：传入要分割的原始字符串；后续调用时：传入`NULL`（表示继续分割上一个字符串）。
    - `delimiters`: 分隔符集合（字符串），包含所有用作分割的字符（如`",;|"`表示逗号、分号、竖线都是分隔符）。
    - `saveptr`: 仅`strtok_r`可用：用于保存上次分割的位置，避免静态变量冲突（线程安全关键）。
- 返回值：
    - 成功：返回当前分割得到的`token`指针（指向子字符串的起始地址）；
    - 失败：当没有更多`token`可分割时，返回`NULL`。
### 工作原理：
- 破坏性修改原始字符串：
    - 分割时会将原始字符串中遇到的第一个分隔符替换为`'\0'`（字符串结束符），从而使当前`token`成为独立字符串；同时记录下一个字符的地址，作为下次分割的起点。
- 状态保存机制：
    - `strtok()`内部使用静态变量保存上次分割的位置，因此第一次调用传入 str 后，后续调用需传入`NULL`,函数会自动从上次记录的位置继续分割；
    - 静态变量是全局共享的，因此多线程环境下使用`strtok()`会导致状态冲突（线程不安全），这也是推荐用`strtok_r`的原因。
- 分隔符的处理规则：
    - 连续的分隔符会被视为 “一个整体”（不会生成空`token`）；
    - 字符串开头 / 结尾的分隔符会被忽略（例如`";;a,b;;c"`分割后只会得到`a、b、c`）。
### 基础用法：
- 1. 多分隔符分割：
```c++
#include <stdio.h>
#include <string.h>

int main() {
    char str[] = "Bob;30|Male,Shanghai";
    const char *delims = ",;|";  // 多个分隔符：逗号、分号、竖线
    
    char *token = strtok(str, delims);
    while (token != NULL) {
        printf("token：%s\n", token);
        token = strtok(NULL, delims);
    }
    
    return 0;
}
```
> 输出：
```
token：Bob
token：30
token：Male
token：Shanghai
```
- 2. 线程安全版`strtok_r`用法:
    -`strtok_r`通过`saveptr`参数手动保存分割状态，避免静态变量冲突，适合多线程或嵌套分割场景：
```c++
#include <stdio.h>
#include <string.h>

int main() {
    char str[] = "Charlie,35;Male|Guangzhou";
    const char *delims = ",;|";
    char *saveptr;  // 用于保存分割位置（线程安全关键）
    
    // 第一次调用：传入 str、delims、&saveptr
    char *token = strtok_r(str, delims, &saveptr);
    while (token != NULL) {
        printf("token：%s\n", token);
        // 后续调用：str 传 NULL，saveptr 保持不变
        token = strtok_r(NULL, delims, &saveptr);
    }
    
    return 0;
}
```     
- 3. 嵌套分割示例（分割 token 中的子串）:
    - 用 strtok_r 可实现嵌套分割（例如先按分号分割，再按竖线分割子串）：
```c++
#include <stdio.h>
#include <string.h>

int main() {
    char str[] = "David;40|Male;Shenzhen|Guangdong";
    const char *delims1 = ";";  // 一级分隔符：分号
    const char *delims2 = "|";  // 二级分隔符：竖线
    char *saveptr1, *saveptr2;  // 两个 saveptr 分别保存两级分割状态
    
    // 一级分割：按分号拆分
    char *token1 = strtok_r(str, delims1, &saveptr1);
    while (token1 != NULL) {
        // 二级分割：如果当前 token 包含竖线，继续拆分
        if (strchr(token1, '|') != NULL) {  // 检查是否包含二级分隔符
            char *token2 = strtok_r(token1, delims2, &saveptr2);
            while (token2 != NULL) {
                printf("子 token：%s\n", token2);
                token2 = strtok_r(NULL, delims2, &saveptr2);
            }
        } else {
            printf("一级 token：%s\n", token1);
        }
        token1 = strtok_r(NULL, delims1, &saveptr1);
    }
    
    return 0;
}
```
> 输出：
```
一级 token：David
子 token：40
子 token：Male
子 token：Shenzhen
子 token：Guangdong
```
### 注意点：
- 修改原始字符串：
`strtok()`会直接修改`str`参数（替换分隔符为`'\0'`），如果后续需要使用原始字符串，必须提前复制
- 线程不安全：
`strtok()`内部的静态变量会导致多线程冲突，例如两个线程同时调用 strtok() 分割不同字符串，会出现 `token`错乱。解决方案：全程使用`strtok_r（POSIX）或 strtok_s（Windows）`。
- 连续分隔符导致空`token`丢失：
`strtok()`会忽略连续的分隔符，例如`"a,,b;;c"`分割后得到`a、b、c，`不会产生空`token`。如果需要保留空`token`（如 CSV 中空字段），strtok() 无法直接实现，需手动处理（或使用自定义分割函数）。
- 嵌套调用`strtok()`覆盖状态：
如果在一个`strtok()`循环中嵌套另一个`strtok()`调用，会覆盖内部静态变量的状态，导致外层分割中断。解决方案：用`strtok_r`手动管理`saveptr`，避免状态冲突。
- C++ 的`std::string`不能直接传给`strtok()`，但可以通过间接方式适配使用—— 核心原因是`strtok()` 的设计依赖 C 风格字符串`（char*）`，且会修改原始字符串，而`std::string`是 C++ 封装的字符串类，两者本质不同。
    - 解决方法：
```c++
#include <iostream>
#include <string>
#include <cstring>  // 包含 strtok()

int main() {
    std::string str = "apple,banana,orange,grape";  // C++ string
    const char* delims = ",";                       // 分隔符集合

    // 1. 复制 string 内容到可修改的 char 数组（需确保数组长度足够）
    char* c_str = new char[str.size() + 1];  // +1 留足 '\0' 位置
    strcpy(c_str, str.c_str());              // 复制 string 的 C 风格字符串

    // 2. 用 strtok() 分割
    char* token = strtok(c_str, delims);
    while (token != NULL) {
        std::cout << "token: " << token << std::endl;
        token = strtok(NULL, delims);
    }

    // 3. 释放动态分配的内存（避免内存泄漏）
    delete[] c_str;
    return 0;
}
```
-----------
```c++
#include <iostream>
#include <string>
#include <vector>
#include <cstring>

int main() {
    std::string str = "cat;dog|rabbit;fish";
    const char* delims = ";|";

    // 1. 用 vector<char> 存储可修改的字符串（自动管理内存）
    std::vector<char> c_str(str.begin(), str.end());
    c_str.push_back('\0');  // 手动添加字符串结束符

    // 2. 分割（vector.data() 返回 char*，可修改）
    char* token = strtok(c_str.data(), delims);
    while (token != NULL) {
        std::cout << "token: " << token << std::endl;
        token = strtok(NULL, delims);
    }

    return 0;  // vector 自动释放内存，无需手动操作
}
```

## 7. strcpy :
### 标准原型:
&emsp;&emsp;`char *strcpy(char *dest, const char *src);`
- 参数：
>|参数|作用|注意事项|
>|:---|:---|:---|
>|`dest`|目标字符串（接收复制内容的缓冲区），类型为`char*`（必须可修改）。|1. 必须是可修改的字符数组（char[]）或动态分配的 char*；<br> 2. 缓冲区大小必须 ≥ 源字符串长度 + 1（预留`'\0'`位置）。|
>|`src`|源字符串（被复制的内容），类型为`const char*`（只读，不被修改）。	|可以是字符串常量（如 "hello"）、字符数组或`const char*` 指针，必须以`'\0'`结尾（否则会越界访问）。|
- 返回值:
返回`dest`的指针（即目标字符串的起始地址），方便链式调用（如`printf("%s", strcpy(dest, src))`）。
### 核心工作原理
- 从`src`的起始地址开始，逐个字节复制字符到`dest`对应的地址，直到遇到 `src` 中的 `'\0'`；
- 会将 `src` 的 '\0' 一并复制到 `dest` 中（确保 dest 是合法的 C 风格字符串）；
- 不检查` dest `的缓冲区大小：如果 `dest` 空间不足，会导致 “缓冲区溢出”（写入超出 `dest` 范围的内存），引发未定义行为（程序崩溃、内存错乱等）。
### 示例
- 字符数组间复制
```c++
#include <stdio.h>
#include <string.h>

int main() {
    char src[] = "Hello, C!";  // 源字符串（字符数组，可修改）
    char dest[20];             // 目标字符串（缓冲区大小 20，足够容纳 src）

    // 复制 src 到 dest
    strcpy(dest, src);

    // 输出结果：dest 内容与 src 完全一致
    printf("源字符串：%s\n", src);
    printf("目标字符串：%s\n", dest);
    printf("dest 地址：%p，返回值地址：%p\n", dest, strcpy(dest, src));  // 返回值是 dest 指针

    return 0;
}
```
> 输出：
```
源字符串：Hello, C!
目标字符串：Hello, C!
dest 地址：0x7ffee4b7e960，返回值地址：0x7ffee4b7e960
```
- 字符串常量复制到字符数组:
```c++
#include <stdio.h>
#include <string.h>

int main() {
    const char *src = "I love programming!";  // 源字符串：字符串常量（只读）
    char dest[30];                            // 目标缓冲区足够大

    strcpy(dest, src);
    printf("复制结果：%s\n", dest);  // 输出：复制结果：I love programming!

    return 0;
}
```
- 链式调用（利用返回值）:
```c++
#include <stdio.h>
#include <string.h>

int main() {
    char dest[20];
    // strcpy 返回 dest 指针，直接传给 printf 输出
    printf("链式调用结果：%s\n", strcpy(dest, "链式调用成功！"));

    return 0;
}
```
### 注意
> strcpy() 的最大风险是不检查目标缓冲区大小，容易引发缓冲区溢出。
- 目标缓冲区大小不足（最常见）:
```c++
#include <stdio.h>
#include <string.h>

int main() {
    char src[] = "这是一个很长的字符串，远超 dest 的大小";
    char dest[10];  // 缓冲区仅 10 字节，根本容纳不下 src

    strcpy(dest, src);  // 缓冲区溢出！未定义行为（程序可能崩溃、乱码）
    printf("dest：%s\n", dest);

    return 0;
}
```




### 

# others
## 1. memset
### 函数原型
```c++
#include <string.h>

void *memset(void *ptr, int value, size_t num);
```
- 参数说明：

>|参数|	含义|
>|:---|:---|
>|`ptr`|	指向要初始化的内存块的指针（可以是任意类型指针，如 char*、int* 等）|
>|`value`	|要设置的「字节值」（虽然声明为 int，但实际只使用低 8 位，取值范围 0~255）|
>|`num`	|要设置的字节数（不是元素个数！）|

- 返回值：返回指向原始内存块 ptr 的指针（方便链式调用）。
### 特性
- **按字节赋值**：无论 `ptr` 指向什么类型（`int、struct`等），`memset` 都会逐个字节设置为 `value` 的低 8 位
- **覆盖原始数据**：直接操作内存，会覆盖目标区域的所有原有数据
- **效率高**：底层由编译器优化（通常是汇编指令），比手动循环赋值更快
- **无类型依赖**：不关心 `ptr` 的具体类型，仅操作内存字节，通用性强

### 常见用法
- **初始化字符数组（最安全场景）**
    - 字符数组的每个元素就是 1 字节，与 `memset` 的「按字节赋值」完全匹配，是最常用且无风险的场景
```c++
#include <stdio.h>
#include <string.h>

int main() {
    char str[20];
    // 将 str 的前 10 个字节设为 'a'（ASCII 97），其余字节未初始化
    memset(str, 'a', 10);
    str[10] = '\0'; // 手动添加字符串结束符
    printf("%s\n", str); // 输出：aaaaaaaaaa

    // 清空字符数组（设为 0，即 '\0'）
    memset(str, 0, sizeof(str));
    printf("%s\n", str); // 输出空字符串
    return 0;
}
```
- **清空整数数组（仅适用于设为 0或-1）**
     - 整数（如 `int`）通常占 4 字节（32 位系统），`memset` 会将每个字节都设为 `value`。因此：
        - 只能用 `memset` 将整数数组设为 0（4 个字节都是 0，整体就是 0）。
        - 不能用 `memset` 设为其他值（如 1），否则每个字节都是 1，实际值为 0x01010101（十进制 16843009），而非 1。
```c++
#include <stdio.h>
#include <string.h>

int main() {
    int arr[5];
    // 正确：将 arr 所有字节设为 0（每个 int 变为 0x00000000）
    memset(arr, 0, sizeof(arr)); // sizeof(arr) = 5*4=20 字节
    for (int i = 0; i < 5; i++) {
        printf("%d ", arr[i]); // 输出：0 0 0 0 0
    }
    printf("\n");

    // 错误：想设为 1，但实际每个 int 是 0x01010101（16843009）
    memset(arr, 1, sizeof(arr));
    for (int i = 0; i < 5; i++) {
        printf("%d ", arr[i]); // 输出：16843009 16843009 ...
    }
    return 0;
}
```
- **初始化结构体 / 自定义类型**
    - 可快速将结构体的所有成员（包括填充字节）设为 0，适合清空状态。
```c++
#include <stdio.h>
#include <string.h>

typedef struct {
    int id;
    char name[20];
    float score;
} Student;

int main() {
    Student s;
    // 将结构体 s 的所有字节设为 0
    memset(&s, 0, sizeof(Student));
    printf("id: %d, name: %s, score: %.1f\n", s.id, s.name, s.score);
    // 输出：id: 0, name: , score: 0.0
    return 0;
}
```
- **填充缓冲区**
    - 网络编程、文件操作中，常用 memset 初始化缓冲区（如设为 0 或特定填充符）。
```c++
#include <string.h>

#define BUF_SIZE 1024

int main() {
    char buf[BUF_SIZE];
    // 清空缓冲区（避免残留垃圾数据）
    memset(buf, 0, BUF_SIZE);
    // 或填充特定字节（如 0xFF，用于二进制数据）
    // memset(buf, 0xFF, BUF_SIZE);
    return 0;
}
```
### 注意事项
- `value`是字节值，仅低 8 位有效：
    - 若传入 `value` = 256，低 8 位为 0，实际效果等同于 `value` = 0；
    - 若需设置非 0 字节（如 0xAB），直接传入十六进制值即可（`memset(ptr, 0xAB, num)`）。
- `num` 是字节数，不是元素个数：
    - 错误：`int arr[5]; memset(arr, 0, 5);`（仅设置前 5 字节，即第一个 int 的前 1 字节，其余元素未初始化）；
    - 正确：`memset(arr, 0, sizeof(arr));` `（sizeof(arr)` 计算总字节数，适配不同类型）。
- 避免越界访问：
    - ptr 指向的内存块必须至少有 num 字节，否则会触发未定义行为（覆盖非法内存，可能导致程序崩溃）。
    - 示例：`char str[10]; memset(str, 'a', 20);`（越界，危险！）。
-  不适合非字节对齐的类型赋值：
    - 除了「设为 0」，memset 不能用于其他值的非字符类型初始化（如 short、long、double 等），因为这些类型占多个字节，按字节填充会导致值错误。
- `ptr` 不能是 `NULL`：
    - 若 `ptr` 为 `NULL`，调用 `memset` 会直接崩溃（空指针解引用），需提前确保 `ptr` 指向有效内存。
- `volatile` 变量的特殊处理：
    - 若 `ptr` 指向 `volatile` 修饰的内存（如硬件寄存器），`memset` 可能被编译器优化失效，需用 `volatile` 指针或手动循环赋值。


## 2. count
### header
&emsp;&emsp;`#include <algorithm> `
### grammmar
```c++
#include <algorithm>  // 必须包含的头文件

// 统计 [first, last) 区间内 == value 的元素个数
template <class InputIterator, class T>
typename iterator_traits<InputIterator>::difference_type
count(InputIterator first, InputIterator last, const T& value);
```
- 参数：`first`（起始迭代器）、`last`（结束迭代器，左闭右开）、`value`（要统计的值）；
- 返回值：匹配元素的个数（类型是`difference_type`，通常等价于`int`）。
### examples
- （1）统计 `vector` 中元素出现次数:
```c++
#include <iostream>
#include <vector>
#include <algorithm>  // std::count

int main() {
    std::vector<int> vec = {1, 2, 3, 2, 4, 2, 5};
    int target = 2;
    
    // 统计 vec 中 2 出现的次数（依赖 begin()/end() 迭代器）
    int cnt = std::count(vec.begin(), vec.end(), target);
    std::cout << target << " 出现了 " << cnt << " 次" << std::endl;  // 输出：2 出现了 3 次
    
    return 0;
}
```

- （2）统计数组中元素出现次数
```c++
#include <iostream>
#include <algorithm>
#include <cstring>  // strlen

int main() {
    char str[] = "abacaba";
    char target = 'a';
    
    // 数组用 指针 作为迭代器（str 是首地址，str + strlen(str) 是末尾）
    int cnt = std::count(str, str + strlen(str), target);
    std::cout << target << " 出现了 " << cnt << " 次" << std::endl;  // 输出：a 出现了 4 次
    
    return 0;
}
```
- （3）统计 string 中字符出现次数
```c++
#include <iostream>
#include <string>
#include <algorithm>

int main() {
    std::string s = "hello world";
    char target = 'l';
    
    int cnt = std::count(s.begin(), s.end(), target);
    std::cout << target << " 出现了 " << cnt << " 次" << std::endl;  // 输出：l 出现了 3 次
    
    return 0;
}
```
### std::count_if
- 统计「满足自定义条件」的元素（而非等于某个固定值）
```c++
#include <iostream>
#include <vector>
#include <algorithm>

// 自定义条件：统计偶数的个数
bool isEven(int x) {
    return x % 2 == 0;
}

int main() {
    std::vector<int> vec = {1, 2, 3, 4, 5, 6};
    
    // 统计满足 isEven 条件的元素个数
    int evenCnt = std::count_if(vec.begin(), vec.end(), isEven);
    std::cout << "偶数的个数：" << evenCnt << std::endl;  // 输出：3
    
    return 0;
}
```


