
<center><font color=pink face='楷体' size=7> 做字符串相关题中遇到的fouctions</font></center>            
<font face='宋体'>

# 1. push_back
> ## functions :
- 向容器尾部追加元素，时间复杂度通常为 O(1)
- 会创建新元素的副本（或移动副本，C++11 后支持右值引用），原元素不会被修改
- 对于 std::vector，若当前容量不足以容纳新元素，会自动分配更大的内存空间，并将原有元素拷贝 / 移动到新空间
> ## range of application :
- vector、queue、list、string
> ## example :
```
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

# 2. getline :
> ## usage forms:
- 全局函数 std::getline()（定义在 <string> 头文件）：读取流（如 cin、文件流）中的一整行到 std::string,不用担心 “输入超长” 的问题
> - grammar:getline(输入流, 接收字符串变量, 终止符)
> - 终止符（可选）：默认是 '\n'（换行符），遇到该字符停止读取 **（终止符不会存入字符串）**
> - 解决 cin >> 无法读取带空格的字符串的问题
```
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
- 类成员函数 istream::getline()（定义在 <iostream> 头文件）：读取流到字符数组（char[]）
> - grammar:输入流.getline(字符数组, 数组大小, 终止符)
> - 终止符（可选）：默认 '\n'，读取到后停止，且会在字符数组末尾自动添加 '\0'（字符串结束符）

```
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

> ## matters:
-  如果先使用 cin >> 读取数据（如整数、单个单词），再用 getline()，会发现 getline() 直接读取到空字符串 —— 因为 cin >> 会留下换行符在输入缓冲区，getline() 会把这个换行符当作 “空行” 读取。
>> solutions :
>> 1. 用 cin.ignore() 清空缓冲区的换行符         
>> - cin.ignore(n, ch) 会忽略输入缓冲区中前 n 个字符，直到遇到 ch（默认忽略 1 个字符）：
```
cin >> age;
cin.ignore();  // 忽略缓冲区中残留的 '\n'（默认忽略 1 个字符）
getline(cin, name);  // 正常读取
```
>> - 若缓冲区可能有多个残留字符（如输入年龄后输入了多余空格），可忽略所有字符直到换行：
```
cin.ignore(numeric_limits<streamsize>::max(), '\n');  // 需要包含 <limits> 头文件
```
>> 2. 用 getline() 读取前先清空缓冲区（兼容多残留字符）
>> - cin >> age;
// 清空缓冲区中所有字符，直到换行
```
while (cin.get() != '\n');  
getline(cin, name);
```
