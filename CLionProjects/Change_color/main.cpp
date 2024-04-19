#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
//////////
using namespace cv;
using namespace std;
//////////Scalar backgroundColor(33,33,33);
//////////Scalar lowerBound(175, 129, 22);
//////////Scalar upperBound(182, 255, 255);
//////////
//////////void changeBackground(const Mat& inputImage, const Scalar& backgroundColor, const Scalar& lowerBound, const Scalar& upperBound)
//////////{
//////////    // 图片转为hsv格式
//////////    Mat hsv;
//////////    cvtColor(inputImage, hsv, COLOR_BGR2HSV);
//////////    imshow("?",hsv);
//////////    // 在指定范围内的变为白色，不在范围内的变为黑色
//////////    Mat mask;
//////////    inRange(hsv, lowerBound, upperBound, mask);
//////////
//////////    imshow("D:/QtProject/mask1.png", mask);
//////////    // 取反操作，抠出人像
//////////    bitwise_not(mask, mask);
//////////    // 创建新的背景图像
//////////    Mat newBackground = Mat::zeros(inputImage.size(), inputImage.type());
//////////    newBackground = backgroundColor;
//////////    // 将原始图像复制到新背景图像中，只保留前景（人像）区域
//////////
//////////
//////////    Mat ele= getStructuringElement(0,Size(7,7));
//////////    morphologyEx(mask,mask,MORPH_OPEN,ele);
//////////    imshow("m",mask);
//////////    inputImage.copyTo(newBackground, mask);
//////////    imshow("New Background Image", newBackground);
//////////
//////////
//////////    //保存图片
//////////   // imwrite("newBackground", newBackground);
//////////
//////////
//////////}
//////////
//////////int main(int argc, char* argv[])
//////////{
//////////    // 显示一张图片
//////////    Mat image = imread("/home/yuuki/3.bmp");
//////////    imshow("1",image);
//////////    // 更换背景
//////////    changeBackground(image, backgroundColor, lowerBound, upperBound);
//////////    // 等待用户按下任意键
//////////    waitKey(0);
//////////
//////////    return 0;
//////////}
//////////int main() {
//////////    for (int x = 0; x < 4; x++)
////////////    Point2f zhondian[i]
//////////       std::cout<<x<<std::endl;
//////////    double aaa;
//////////    aaa=atan2(-1,1);
//////////    std::cout<<aaa<<std::endl;
//////////}









////////#include <stdio.h>
////////int sum=0;
////////
////////int susu(int q)
////////{
////////
////////    sum=sum+q;
////////
////////    return susu(q-1);
////////}
////////
////////int main()
////////{
////////    int N,i,x;
////////    scanf("%d",&N);
//////////    for(i=0; i<N; i++)
//////////    {
//////////        scanf("%d",&x);
//////////        printf("%d\n",fib(x));
//////////        }
////////    susu(N);
////////    printf("%d",sum);
////////}
////////int fib(int x)
////////{
////////    if(x==0)return 0;
////////    else if(x==1)return 1 ;
////////    else
////////        return(fib(x-1)+fib(x-2));
////////#include <stdio.h>
////////#define MAX_SIZE 100
////////int get_matrix(int mtx[][100], int m, int n)//读取一个m行n列的矩阵存放在二维数组mtx[][]中。
////////{
////////    int i, j;
////////    for (i = 0; i < m; i++)
////////        for (j = 0; j < n; j++)
////////            scanf("%d", &mtx[i][j]);
////////    return 0;
////////}
////////int count_average(double arr[], int mtx[][100], int m, int n)//计算m行n列的矩阵mtx[][]各列的平均值，存放到数组arr[]中。
////////{
////////    double sum;    int i, j;
////////    for (i = 0; i < n; i++)
////////    {
////////        sum = 0;
////////        for (j = 0; j < m; j++)
////////            sum =sum+ mtx[j][i];
////////        arr[i] = sum / m;
////////    }
////////    return 0;
////////}
////////int put_array(double arr[], int n)//按照输出格式，输出有n个元素的一维数组arr[]。
////////{
////////    int i;
////////    printf("%lg", arr[0]);
////////    for (i = 1; i < n; i++)
////////        printf(" %lg", arr[i]);
////////    putchar('\n');
////////    return 0;
////////}
////////
//////#include <stdlib.h>
//////#include <stdio.h>
//////#define MAX_STR_LEN 1001
//////int str_len(char s[])
//////{
//////    int i;
//////    for(i=0;s[i]!='0';i++)// s[i]!=0;
//////    ;
//////    return i;
//////}
//////
//////int main()
//////{
//////    int n;
//////    char s[MAX_STR_LEN];
//////    while(gets(s) != NULL)
//////    printf("%d\n", str_len(s));
//////    return 0;
//////}
////#include <stdio.h>
////#include <string.h>
////#include <stdlib.h>
////int find(char str[], char ch)
////{
////    static int i=0;//静态变量
////    int j;
////    for(j=i; j<strlen(str); j++)
////    //从上一次找到相同的字符开始，不需要从头开始
////    {
////        i++;
////          if(str[j] == ch)
////         return j;
////        }
////
////    return -1;
////}
////int main()
////{
////    char str[1001], ch;
////    int pos;
////    gets(str);
////    ch = getchar( );
////    pos = find(str, ch);
////    while (pos != -1)
////    {
////        printf("%d\n", pos);
////        pos = find(str, ch);
////    }
////    return 0;
////}
//#include <stdio.h>
//int is_swapped(int *a , int *b)
//{
//int t;
//if(*a > *b)
//return 1;
//else
//return 0;
//}
//
//
//int main()
//{
//    int a, b;
//    scanf("%d%d", &a, &b);
//    if(is_swapped(&a, &b))
//        printf("%d %d YES", b, a);
//    else
//        printf("%d %d NO", a, b);
//}
//
int main() {
//    for (int x = 0; x < 4; x++)
//    Point2f zhondian[i]
//       std::cout<<x<<std::endl;
    double aaa;
    aaa=atan2(134,256);
    std::cout<<aaa<<std::endl;
}