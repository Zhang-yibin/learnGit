#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    Mat image = imread("14.jpg");
    // 2D 特征点像素坐标，这里是用PS找出，也可以用鼠标事件画出特征点
    vector<Point2d> image_points;
    image_points.push_back(Point2d(152, 92));
    image_points.push_back(Point2d(426, 94));
    image_points.push_back(Point2d(428, 394));
    image_points.push_back(Point2d(126, 380));

    // 画出四个特征点
    for (int i = 0; i < image_points.size(); i++)
    {
        circle(image, image_points[i], 3, Scalar(0, 0, 255), -1);
    }

    // 3D 特征点世界坐标，与像素坐标对应，单位是mm
    std::vector<Point3d> model_points;
    model_points.push_back(Point3d(-42.5f, -42.5f, 0)); // 左上角(-42.5mm,-42.5mm)
    model_points.push_back(Point3d(+42.5f, -42.5f, 0));
    model_points.push_back(Point3d(+42.5f, +42.5f, 0));
    model_points.push_back(Point3d(-42.5f, +42.5f, 0));
    //　注意世界坐标和像素坐标要一一对应

    // 相机内参矩阵和畸变系数均由相机标定结果得出
    // 相机内参矩阵
    Mat camera_matrix = (Mat_<double>(3, 3) << 505.1666120558573, 0, 310.1660811401983,
            0, 488.6105193422827, 249.9495562500046,
            0, 0, 1);
    // 相机畸变系数
    Mat dist_coeffs = (Mat_<double>(5, 1) << 0.1311093141581356, -0.9654453502019078,
            0.001118708198552768, -0.001126801336773416, 2.286059640823912);

    cout << "Camera Matrix " << endl << camera_matrix << endl << endl;
    // 旋转向量
    Mat rotation_vector;
    // 平移向量
    Mat translation_vector;

    // pnp求解
    solvePnP(model_points, image_points, camera_matrix, dist_coeffs, \
		rotation_vector, translation_vector, 0, CV_ITERATIVE);
    // 默认ITERATIVE方法，可尝试修改为EPNP（CV_EPNP）,P3P（CV_P3P）

    cout << "Rotation Vector " << endl << rotation_vector << endl << endl;
    cout << "Translation Vector" << endl << translation_vector << endl << endl;

    Mat Rvec;
    Mat_<float> Tvec;
    rotation_vector.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
    translation_vector.convertTo(Tvec, CV_32F); // 平移向量转换格式

    Mat_<float> rotMat(3, 3);
    Rodrigues(Rvec, rotMat);
    // 旋转向量转成旋转矩阵
    cout << "rotMat" << endl << rotMat << endl << endl;

    Mat P_oc;
    P_oc = -rotMat.inv() * Tvec;
    // 求解相机的世界坐标，得出p_oc的第三个元素即相机到物体的距离即深度信息，单位是mm
    cout << "P_oc" << endl << P_oc << endl;

    imshow("Output", image);
    waitKey(0);
}
