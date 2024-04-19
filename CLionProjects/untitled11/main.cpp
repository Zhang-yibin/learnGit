#include "matplotlibcpp.h"
#include <cmath>
#include "python3.10/pythonrun.h"
namespace plt = matplotlibcpp;

int main()
{
    // 准备数据
    int n = 5000;
    std::vector<double> x(n), y(n), z(n), w(n,2);
    for(int i=0; i<n; ++i) {
        x.at(i) = i*i;
        y.at(i) = sin(2*M_PI*i/360.0);
        z.at(i) = log(i);
    }

    // 设置输出图像的大小为1200x780像素
    plt::figure_size(1200, 780);
    // 绘制给定x和y数据的折线图，颜色自动选择
    plt::plot(x, y);
    // 绘制给定x和y数据的红色虚线
    plt::plot(x, w, "r--");
    // 绘制一条线，其名称将出现在图例中为"log(x)"
    plt::named_plot("log(x)", x, z);
    // 设置x轴的范围为[0,1000000]
    plt::xlim(0, 1000*1000);
    // 添加图表标题
    plt::title("Sample figure");
    // 启用图例
    plt::legend();
    // 保存图像（文件格式由扩展名确定）
    plt::save("./basic.png");

    return 0;
}