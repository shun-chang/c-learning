<font face='微软雅黑' size=10 color=blue><center>Hash Table</font></center>
<font face='微软雅黑'>

[TOC]

# 有关函数
## map(红黑树)
- 有序键对（key-value）映射，key唯一且有序，底层是红黑树，本质是“有序映射”。
- 元素顺序：按key升序排列
- 查找时间复杂度：O(log n)
- 插入/删除时间复杂度：O(log n)
- 内存开销较小
- 核心应用场景：需有序遍历键对值、按范围查询（如找比x大的最小key），例：学生成绩排序存储（学号为key）。
- example:
```c++
#include <iostream>
#include <map>
using namespace std;

int main() {
    map<int, string> student; // key：学号（int），value：姓名（string）

    // 1. 插入
    student.insert({101, "张三"}); // 方式1：键值对
    student[102] = "李四";        // 方式2：下标赋值（不存在则插入，存在则修改value）
    student.emplace(103, "王五"); // 方式3：原地构造（更高效）

    // 2. 查找（按key）
    auto it = student.find(102);
    if (it != student.end()) {
        cout << "找到学号102：" << it->second << endl; // 输出：找到学号102：李四
    }

    // 3. 遍历（默认按key升序）
    cout << "\nmap遍历（有序）：" << endl;
    for (const auto& [id, name] : student) { // 结构化绑定（C++17+）
        cout << "学号：" << id << "，姓名：" << name << endl;
    }

    // 4. 删除（按key）
    student.erase(101);
    cout << "\n删除学号101后大小：" << student.size() << endl; // 输出：2

    // 5. 获取大小
    cout << student.size() << endl;

    return 0;
}
```
## unordered_map(哈希表)
- 无序键值对映射，key唯一但无序，底层是哈希表，本质是“哈希映射”。
- 元素顺序：无序（哈希值排序）
- 查找时间复杂度：平均O(1),最坏O(n)
- 插入/删除时间复杂度：平均O(1),最坏O(n)
- 内存开销：较大
- 核心使用场景：追求高效增删查（无顺序要求），例：统计单词出现次数，缓存映射
- example:
```c++
#include <iostream>
#include <unordered_map>
using namespace std;

int main() {
    unordered_map<string, int> wordCount; // key：单词，value：出现次数

    // 1. 插入
    wordCount["apple"] = 2;
    wordCount.insert(make_pair("banana", 3));
    wordCount.emplace("orange", 1);

    // 2. 查找
    string target = "banana";
    if (wordCount.count(target)) { // count返回0或1（key唯一）
        cout << target << "出现次数：" << wordCount[target] << endl; // 输出：3
    }

    // 3. 遍历（无序）
    cout << "\nunordered_map遍历（无序）：" << endl;
    for (const auto& pair : wordCount) {
        cout << pair.first << "：" << pair.second << "次" << endl;
    }

    // 4. 删除（按迭代器）
    auto it = wordCount.find("orange");
    if (it != wordCount.end()) {
        wordCount.erase(it);
    }

    return 0;
}
```
## set(红黑树)
- 有序唯一元素集合，元素及key（无value），底层是红黑树，本质是“有序集合”
- 按元素升序排序
- 查找时间复杂度：O(log n)
- 插入/删除时间复杂度：O(log n)
- 内存开销较小
- 核心使用场景：需有序存储唯一元素，去重+排序
- example:
```c++
#include <iostream>
#include <set>
using namespace std;

int main() {
    set<int> scores; // 存储考试分数（自动去重+升序）

    // 1. 插入（重复元素自动忽略）
    scores.insert(85);
    scores.insert(92);
    scores.insert(85); // 重复，插入失败
    scores.emplace(78);

    // 2. 查找
    int findScore = 92;
    if (scores.find(findScore) != scores.end()) {
        cout << "找到分数：" << findScore << endl; // 输出：找到分数：92
    }

    // 3. 有序遍历
    cout << "\nset有序遍历：" << endl;
    for (int s : scores) {
        cout << s << " "; // 输出：78 85 92
    }
    cout << endl;

    // 4. 范围查询（红黑树优势）
    auto lower = scores.lower_bound(80); // 第一个>=80的元素（85）
    auto upper = scores.upper_bound(90); // 第一个>90的元素（92）
    cout << "\n80~90之间的分数：" << endl;
    for (auto it = lower; it != upper; ++it) {
        cout << *it << " "; // 输出：85
    }

    return 0;
}
```
## unordered_set(哈希表)
- 无序唯一元素集合，元素及key，底层是哈希表，本质是“哈希集合”
- 元素顺序：无序（哈希值排序）
- 查找时间复杂度：平均O(1),最坏O(n)
- 插入/删除时间复杂度：平均O(1),最坏O(n)
- 内存开销：较大
- 核心使用场景：仅需去重，无顺序需求，例：快速判断元素是否在集合中（如查重）
- example:
```c++
#include <iostream>
#include <unordered_set>
using namespace std;

int main() {
    unordered_set<string> names; // 存储不重复姓名（无序）

    // 1. 插入
    names.insert("Alice");
    names.insert("Bob");
    names.insert("Alice"); // 重复忽略
    names.emplace("Charlie");

    // 2. 判重（核心用途）
    string checkName = "Bob";
    if (names.count(checkName)) {
        cout << checkName << "已存在" << endl; // 输出：Bob已存在
    }

    // 3. 遍历（无序）
    cout << "\nunordered_set遍历（无序）：" << endl;
    for (const auto& name : names) {
        cout << name << " "; // 输出顺序不固定（如：Bob Alice Charlie）
    }
    cout << endl;

    // 4. 删除（按key）
    names.erase("Charlie");
    cout << "删除Charlie后大小：" << names.size() << endl; // 输出：2

    // 5. 获取大小
    cout << names.size() << endl;

    return 0;
}
```