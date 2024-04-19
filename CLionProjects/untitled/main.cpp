#include <iostream>
using namespace std;

//声明Time类
class Time
{
public:  //成员函数共有部分
    Time()  //定义构造成员函数，函数名与类名相同
    {
        hour= 0;  //利用构造函数给对象中的数据成员都赋初值为0
        minute= 0;
        sec= 0;
    }
    //成员函数的声明
    void set_time();
    void show_time(void);
private:  //类的私有数据部分
    int hour;  //默认数据也是私有的
    int minute;
    int sec;
};

//定义成员函数
//获取时间数据函数
void Time::set_time(void)
{
    cin >> hour;
    cin >> minute;
    cin >> sec;
}

//显示时间格式的函数
void Time::show_time(void)  //显示时间函数
{
    cout << hour << ":" << minute << ":" << sec << endl;
}

//主函数
int main()
{
    Time t1;  //通过类Time实例化对象t1
    t1.set_time();  //调用成员函数，给t1的数据成员赋值
    t1.show_time();  //显示t1的数据成员值
    return 0;
}
