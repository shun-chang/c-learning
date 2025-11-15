# <center>c++知识的学习</center>

<font face='宋体'>
[toc]

## 一、类的继承     
### 1.定义
> 继承机制是面向对象程序设计中使用代码可以复用的最重要的手段，它允许程序员在保持原有类特性的基础上进行扩展，增加功能。这样产生的新类，称为派生类（或子类），被继承的类称为基类（或父类）。
### 2.格式
> class 新类的名字 **：** 继承方式  继承类的名字{ }；如：`class student：public human{ }；`
### 3.子类成员访问权限
- 基类`private`成员无论以什么方式继承到派生类中都是不可兼得。
-  基类`private`成员在派生类中不能被访问，如果基类成员不想在派生类外直接被访问，但需要在派生类中访问，就定义为`protected`。可以看出保护成员限定符是因继承才出现的。
-  使用关键字`class`时默认的继承方式是`private`（及继承为子类的私有成员），使用`struct`时默认的继承方式是`public`（及继承为子类的公有成员），但最好显示地写出继承方式。 
### 4.继承方式  
- 公有继承、保护继承、私有继承
  ```c++
  class A{
    public:
    int a;
    protected:
    int b;
    private:
    int c;
  };


  class B : public  A{
    public:
    int a;
    protected:
    int b;
    不可访问：
    int c;
  };

  class C :protected A{
    protected:
    int a;
    int b;
    不可访问：
    int c;
  };

  class D : private A{
    private:
    int a;
    int b;
    不可访问：
    int c;
  };
  ```

### 5.子类（派生类）和父类（基类）
- 赋值：            

  1. 限制：正常情况下，子类可以赋值给父类，父类不可以赋值给子类
  2. 方式：
  - `=` 符号
  ```c++
  student st;//子类
  human hm;//父类
  hu=st;
  ```
  - 引用
  ```c++
  student st;//子类
  human& hm=st;//父类
  ```
  - 指针
  ```c++
  student st;//子类
  human* hm=&st;//父类
  ```
- 同名的成员变量：

  - 通常情况下编辑器是以子类优先，如下：
  ```c++
    class human
    {
    public:
	    string name = "小明";
    };
    class student :public human
    {
    public:
	    string name = "小红";
	    void print()
	    {
	    	cout << name << endl;
	    }
    };
    int main()
    {
	    student st;
	    st.print();
	    return 0;
    }
  ```
  - 此时会输出小红，如果想访问父类的该成员变量，就需要加上修饰：
  ```c++
  void print(){
    cout <<human::name<<endl;
  }

  或者在 main 函数中加上 cout<<st.human::name;

  ```
- 同名成员函数：
  
  - 通常情况下编辑器是以子类优先，如下：
  ```c++
  class human {
    public:
        string name = "小明";
        void print()
        {
            cout << name << endl;
        }
    };

    class student :public human {
    public:
         string name = "小红";
          void print()
         {
            cout << name << endl;
         }
    };
    int main()
    {
        student st;
         st.print();
         return 0;
    }
    ```
    - 若要调用父类里的同名函数，操作同上，需要给出父类的作用域


### 6.构造和析构
- 构造函数：
  
  - 函数名与类名一致
  - 无返回值
  - 不能用`const` `static` `virtual`等修饰符修饰
  - 类型：
  
  1. 默认构造函数：
  > 无需传入参数即可调用的构造函数，如：
  ```c++
  class MyString {
    public:
         MyString() = default; // 显式生成默认构造函数
    private:
        char* str_ = nullptr; // 类内初始值
        size_t len_ = 0;
    };
    MyString s; // 调用默认构造函数，s.str_=nullptr，s.len_=0
  ```
  2. 带参构造函数：
   - 按输入参数初始化对象，支持灵活创建对象
   - 单参数构造函数建议用 **`explicit`** 修饰，禁止隐式类型转换（避免意外错误）
  
  ```c++
    class MyString {
    public:
         // 单参数构造函数（explicit 禁止隐式转换）
        explicit MyString(size_t len) : len_(len) {
            str_ = new char[len + 1]{}; // 分配资源 + 初始化
        }   
        // 多参数构造函数（C字符串初始化）
        MyString(const char* str) {
            len_ = strlen(str);
            str_ = new char[len_ + 1]{};
            strcpy(str_, str);
        }
    private:
         char* str_ = nullptr;
        size_t len_ = 0;
    };
    MyString s1(10);       // 正确：显式调用单参数构造
    MyString s2("hello");  // 正确：调用多参数构造
    // MyString s3 = 10;   // 错误：explicit 禁止隐式转换
  ```
  3. 拷贝构造函数：
   - 定义：用已有对象初始化新对象，语法：`类名(const 类名& 源对象)`
   - 示例：
  ```c++
    MyString(const MyString& other) : len_(other.len_) {
    str_ = new char[len_ + 1]{}; // 分配独立内存
    strcpy(str_, other.str_);    // 拷贝资源内容
    }
    MyString s1("hello");
    MyString s2 = s1; // 调用拷贝构造，s2 拥有独立内存
  ```
  - 初始化列表：
  
   1. 语法：`构造函数(参数) : 成员1(值1), 成员2(值2) { 函数体 }`
   2. 优势：比函数体内赋值更高效（直接初始化成员，避免 “默认初始化 + 赋值” 的冗余），const 成员、引用成员必须用初始化列表初始化
   3. 示例：
   ```c++
   class Person {
    public:
         // 初始化列表初始化成员（推荐）
        Person(const std::string& name, int age) : name_(name), age_(age) {}
    private:
        const std::string name_; // const 成员必须用初始化列表
        int age_;
    };
  ```
- 析构函数：

  - 类名前加`~`（`~Mystring()`）
  - 无返回值
  - 可加`virtual`(用于析构场景)，不能用`const` `static`
  - 不可重载，及一个类只能有一个析构函数
  - 核心职责：
   1. 释放构造函数中分配的动态资源（如`new[]`分配的内存、`fopen`打开的文件）
   2. 清理对象状态（如关闭网络连接、释放锁资源）
   3. 示例：
   ```c++
   ~MyString() {
        delete[] str_; // 释放动态数组（匹配 new[]）
    }
   ```
   - 匹配`new/new[]`与`delete/delete[]`
     - 动态分配单个对象用`new`→析构用`delete`
     - 动态分配数组用`new[]`→析构用`delete[]`（否则导致内存泄漏或崩溃）
- **继承中的构造和析构顺序**：
  
  - 先构造父类，再构造子类
  - 先析构子类，再析构父类
  
### 7.菱形继承和虚继承：
- 继承的分类
  - 单继承：一个子类只有一个直接父类时称这个继承关系为单继承
  - 多继承：一个子类有两个或以上直接父类时称这个继承关系为多继承
  - 菱形继承: 菱形继承是多继承的一种特殊情况
    - 示例：
    ```c++
    class Person
    {
    public:
	    string _name; // 姓名
    };
 
    class Student : public Person
    {
    protected:
	    int _num; //学号
    };
 
    class Teacher : public Person
    {
    protected:
    	int _id; // 职工编号
    };
 
    class Assistant : public Student, public Teacher
    {
    protected:
    	string _majorCourse; // 主修课程
    };
    int main()
    {
	    Assistant a;
	    a._name = "peter"; // 这样会有二义性无法明确知道访问的是哪一个
 
	    // 需要显示指定访问哪个父类的成员可以解决二义性问题，但是数据冗余问题无法解决
         // 如果Person类中的数据量很大，那么数据冗余量也会很大
	    a.Student::_name = "张三";
	    a.Teacher::_name = "李四";
         return 0;
    }
    ```
- 虚继承（virtual）：
   - 虚拟继承可以解决菱形继承的二义性和数据冗余的问题，虚继承关键字：`virtual`,如上面的继承关系，在Student和Teacher的继承Person时使用虚拟继承，即可解决问题。
   - 注意：虚继承不能在其他地方去使用，只能用来解决菱形继承问题
  ```c++
  class Person
    {
    public:
	    string _name; // 姓名
    };
 
    class Student : virtual public Person // 在会存在菱形继承的地方使用虚继承
    {
    protected:
	    int _num; //学号
    };
 
    class Teacher : virtual public Person
    {
    protected:
	    int _id; // 职工编号
    };
 
    class Assistant : public Student, public Teacher
    {
    protected:
	    string _majorCourse; // 主修课程
    };
 
 
    int main()
    {
	    Assistant a;
	    a._name = "张三";
	    return 0;
    }
  ```



## 二、抽象类
### 1. 形式
- 声明了虚函数的类
- 只定义了protected型构造函数的类
### 2.纯虚函数
- 纯虚函数是一种特殊的虚函数，在许多情况下，在基类中不能对虚函数给出有意义的实现，而它声明为纯虚函数,**它的实现留给该基类的派生类（子类）去做**。这就是纯虚函数的作用。
  
  - 格式：
  ```c++
  class 类名 {
    public:
    virtual 返回类型 函数名（参数 ...） = 0;
  }
  ```
  - 例子：
  ```c++
  class A
    {
    public:
        virtual void print() = 0;
        virtual void display() = 0;
    };
 
    class B
    : public A
    {
    public:
        virtual void print() override{
            cout << "B::print()" << endl;
        }
    };
 
    class C
    : public B
    {
    public:
        virtual void display() override{
            cout << "C::display()" << endl;
        }
    };
 
    void test0(){
        //A类定义了纯虚函数，A类是抽象类
        //抽象类无法创建对象
        //A a;//error
  
        //B b;//error
        C c;
        A * pa2 = &c;
        pa2->print();
        pa2->display();
    }
    ```

>在A类中声明纯虚函数，A类就是抽象类，无法创建对象；
>在B类中去覆盖A类的纯虚函数，如果把所有的纯虚函数都覆盖了（都实现了），B类可以创建对象；只要还有一个纯虚函数没有实现，B类也会是抽象类，也无法创建对象；
>再往下派生C类，完成所有的纯虚函数的实现，C类才能够创建对象。 
- 最顶层的基类（声明纯虚函数的类）虽然无法创建对象，但是可以定义此类型的指针，指向派生类对象，去调用实现好的纯虚函数.

## 三、数据抽象
### 1. 关键点
- 抽象的目标：
  
  - 隐藏实现细节：
       - 用户不需要知道类的内部数据和具体实现。
  - 提供简单的接口：
       - 用户可以通过类提供的接口（如成员函数）操作对象。
  - 提高代码的可维护性
- 实现方式：
  - 使用访问控制符：
   1. `private`:隐藏类的内部数据，只有类的成员函数可以访问。
   2. `public`:暴露类的接口，供外部使用。
  - 定义**成员函数**作为操作接口:
   1. 如`get`和`set`函数
### 2. 与封装的区别
|特性|数据抽象|封装|
|:---|:--|:---|
|定义|隐藏实现细节，只暴露接口|将数据和操作封装在一个单独的单元中（类）|
|目的|提高代码的可读性和灵活性|提高代码的安全性和模块化|
|实现方式|使用访问控制符（`private` 和 `public`）隐藏数据|	使用类将数据和操作结合在一起|
|关注点|关注“隐藏”和“接口”|关注“组合”和“保护”|
### 3. 高级应用
- 抽象类与接口
    - 数据抽象可以通过抽象类与接口组合实现。抽象类定义了行为的框架，派生类实现具体的细节。
    - 示例：
    ```c++
    #include <iostream>
    using namespace std;
 
    class Animal {
    public:
        virtual void makeSound() = 0; // 纯虚函数
        virtual ~Animal() {}
    };
 
    class Dog : public Animal {
    public:
        void makeSound() override {
        cout << "Dog says: Woof!" << endl;
        }
    };
 
    class Cat : public Animal {
    public:
        void makeSound() override {
            cout << "Cat says: Meow!" << endl;
        }
    };
 
    void playSound(Animal* animal) {
        animal->makeSound();
    }
 
    int main() {
        Dog dog;
        Cat cat;
 
        playSound(&dog); // 输出: Dog says: Woof!
        playSound(&cat); // 输出: Cat says: Meow!
 
        return 0;
    }
    ```
    - 说明：
    1. `Animal`是抽象类，定义了`makeSound()`接口。
    2. `Dog`和`Cat`类实现了接口的具体细节。
### 4.实际意义
>- 优点：
      1. 提高安全性：通过隐藏数据成员，防止外部非法访问。
      2. 增强可维护性：实现细节与接口分离，修改内部实现不会影响外部代码。
      3. 提高可读性：通过清晰的接口定义，便于理解和使用类。
      4. 支持代码复用：通过继承和接口，可以在不同场景中复用抽象逻辑。
   
> - 应用场景：
      1. 封装复杂逻辑：隐藏复杂的实现细节，只提供简单的接口。
      2. 模块化设计：将模块的实现细节与使用者分离。
      3. 接口设计：在大型软件中，通过抽象类定义统一接口。
## 四、数据封装
### 1.概念
> - C++中的数据封装是一种面向对象编程（OOP）的核心概念，它指的是将对象的数据（属性）和操作这些数据的方法（行为）捆绑在一起，形成一个独立的单元，即类。通过这种方式，对象的数据被隐藏和保护起来，只能通过对象提供的公共接口（即成员函数）进行访问和修改。这有助于减少程序中各部分的耦合度，提高程序的可维护性和安全性。
### 2.实现机制
- 访问控制符：C++提供了三种访问控制符来控制类的成员（包括数据成员和成员函数）的访问权限，分别是`public`、`protected`和`private`。
   - `public`：成员可以被任意实体访问。
   - `protected`：成员可以被类的成员函数、派生类（子类）的成员函数和友元函数访问，但不能被类的外部直接访问。
   - `private`：成员只能被类的成员函数和友元函数访问，类的外部和派生类都无法直接访问。
  > 在数据封装中，数据成员通常被声明为`private`或`protected`，以限制外部代码直接访问它们，而操作这些数据的方法（成员函数）则被声明为`public`，以提供公共接口。
- 构造函数和析构函数：构造函数用于在创建对象时初始化对象的状态，而析构函数用于在对象销毁前执行清理工作。通过这两个特殊成员函数，可以控制对象的生命周期和状态变化，进一步实现数据封装。
- 成员函数：成员函数是类中定义的用于操作对象数据的函数。通过将数据操作封装在成员函数内部，可以隐藏数据表示的细节，仅通过公共接口暴露必要的功能。
- 示例：
```c++
#include <iostream>

class Rectangle {
private:
    double width;  // 私有数据成员，宽度
    double height; // 私有数据成员，高度

public:
    // 构造函数
    Rectangle(double w, double h) : width(w), height(h) {}

    // 设置宽度
    void setWidth(double w) {
        width = w;
    }

    // 设置高度
    void setHeight(double h) {
        height = h;
    }

    // 获取面积
    double getArea() const {
        return width * height;
    }

    // 获取周长
    double getPerimeter() const {
        return 2 * (width + height);
    }
};

int main() {
    Rectangle rect(10.0, 5.0); // 创建一个Rectangle对象

    std::cout << "Area: " << rect.getArea() << std::endl;
    std::cout << "Perimeter: " << rect.getPerimeter() << std::endl;

    // 尝试直接访问私有数据成员（会编译失败）
    // std::cout << "Width: " << rect.width << std::endl; // 错误：'width' 是私有成员

    return 0;
}
```