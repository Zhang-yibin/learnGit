//
// Created by eski on 6/29/22.
//

#include "detector.h"
#include "classifier.h"
string model_path = "/home/yukki/Downloads/ros2_ws/install/armor_detector/share/armor_detector/model/mlp.onnx";
string label_path = "/home/j11218cpu/Desktop/SR_Vision3.0/Autoaim/Classifier/model/qwq.txt";

//string model_path = "../Autoaim/Classifier/model/mlp1.onnx";
//string label_path = "../Autoaim/Classifier/model/qwq.txt";

double athres = 0.5;
NumberClassifier nc(model_path,label_path,athres);
Mat green_channel;

cv::Mat Detector::preprocessImage(const cv::Mat & img)
{

#ifdef LOOSE_PROCESS
    debug = img.clone();
    vector<Mat> channels;
    cv::Mat binary_img;
    cv::cvtColor(img, binary_img, cv::COLOR_RGB2GRAY);
    cv::threshold(binary_img, binary_img, light_param.binary_threshold, 255, cv::THRESH_BINARY);

#ifdef SHOW_LIGHTS
    //    debug1=img.clone();
    armor_debug = binary_img.clone();
    cvtColor(armor_debug,armor_debug,COLOR_GRAY2BGR);
#endif
#ifdef SHOW_ARMORS
    light_debug = binary_img.clone();
    cvtColor(light_debug,light_debug,COLOR_GRAY2BGR);
#endif
#ifdef SHOW_BINARY(LOOSE_PROCESS)
    imshow("binary",binary_img);
#endif

    return binary_img;
#endif

#ifdef STRICT_PROCESS
    auto time_sstart = std::chrono::steady_clock::now();
        auto time_cap = std::chrono::steady_clock::now();
    auto time = (std::chrono::duration<double,std::milli>(time_cap - time_sstart).count());
    cout<<"time::::::::::::::::"<<time<<endl;
    cv::Mat gray_img,sub_img,output_img;
    green_channel.create(img.size(), CV_8UC1);
    sub_img.create(img.size(), CV_8UC1);
    output_img = cv::Mat(img.size(), CV_8UC1, cv::Scalar(0));

    std::vector<cv::Mat> channels;
    cv::split(img, channels);
    green_channel = channels[1];

    if (detect_color == BLUE) {
        cv::subtract(channels[0], channels[2], sub_img);
    }
    else{
        cv::subtract(channels[2], channels[0], sub_img);
    }

    cv::threshold(green_channel, green_channel, 85, 255, THRESH_BINARY);
    cv::cvtColor(img, gray_img, COLOR_BGR2GRAY);
    cv::threshold(gray_img, gray_img, 40, 255, THRESH_BINARY);
    cv::threshold(sub_img, sub_img, 40, 255, THRESH_BINARY);
    dilate(sub_img, sub_img, kernel3);
    morphologyEx(sub_img,sub_img,MORPH_OPEN,kernel3);

    output_img = sub_img & gray_img;

    dilate(output_img, output_img, kernel3);
    auto time_cap = std::chrono::steady_clock::now();
    auto time = (std::chrono::duration<double,std::milli>(time_cap - time_sstart).count());
    cout<<"time::::::::::::::::"<<time<<endl;
#ifdef SHOW_PREPROCESS_IMG
    cv::imshow("green_channel", green_channel);
    cv::imshow("sub_img", sub_img);
    cv::imshow("output_img", output_img);
#endif

    return output_img;

#endif


}


std::vector<Light> Detector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img)
{


    Mat final_img = binary_img.clone();
    std::vector<std::vector<cv::Point2i> > contours;
    std::vector<cv::Vec4i> hierarchy;
#ifdef STRICT_PROCESS
    // find contour in (sub image of blue and red) & green
    cv::findContours(binary_img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point2i> >::const_iterator it = contours.begin();
    final_img = cv::Mat::zeros(binary_img.size(), CV_8UC1);
    while (it != contours.end())
    {
        cv::Rect rect = cv::boundingRect(*it);
        cv::RotatedRect Rect = cv::minAreaRect(*it);
        // use lights physical condition to choice

        if ((rect.height < 4)||
            (( Rect.size.width > Rect.size.height) && (Rect.angle > light_param.light_deviation -90 && Rect.angle <= 0)) ||
            (( Rect.size.height > Rect.size.width) && (Rect.angle >= -90 && Rect.angle < -light_param.light_deviation))) {
            ++it;
            continue;
        }
        const int margin_y = 5;
        const int margin_x = 3;
        int max_i = rect.x + rect.width, min_i = rect.x;
        max_i = std::min(binary_img.cols, max_i + margin_x);
        min_i = std::max(0, min_i - margin_y);
        int max_j = rect.y + rect.height, min_j = rect.y;
        max_j = std::min(binary_img.rows, max_j + margin_y);
        min_j = std::max(0, min_j - margin_y);

        const uchar *ptr_gray_base = green_channel.data;
        for (size_t j=min_j; j<max_j; ++j) {
            const uchar *ptr_gray = ptr_gray_base + j*green_channel.cols;
            for (size_t i=min_i; i<max_i; ++i) {
                if (*(ptr_gray+i) < 1)
                    continue;
                final_img.at<uchar>(j, i) = 255;
            }
        }
        ++it;
    }
//    erode(final_img,final_img,getStructuringElement(MORPH_RECT,Size(3,3)));
//    dilate(final_img,final_img,getStructuringElement(MORPH_RECT,Size(3,3)));
    morphologyEx(final_img,final_img,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(3,3)));
#ifdef SHOW_ARMORS(STRICT_PROCESS)||SHOW_LIGHTS(STRICT_PROCESS)
    imshow("final",final_img);
    armor_debug=final_img.clone();
    light_debug=final_img.clone();
    cvtColor(light_debug,light_debug,COLOR_GRAY2BGR);
    cvtColor(armor_debug,armor_debug,COLOR_GRAY2BGR);
#endif

#endif
    // find contours in processed pic
    using std::vector;
    cv::findContours(final_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    vector<Light> lights;
    for (const auto & contour : contours) {

        if (contour.size() < 0.5) continue;

        auto r_rect = cv::minAreaRect(contour);
        auto light = Light(r_rect);

        if (isLight(light)) {

            auto rect = light.boundingRect();

            if (  // Avoid assertion failed
                    0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
                    0 <= rect.height && rect.y + rect.height <= rbg_img.rows) {
                int sum_r = 0, sum_b = 0;
                auto roi = rbg_img(rect);
                // Iterate through the ROI
//                for (int i = 0; i < roi.rows; i++) {
//                    auto * data=roi.ptr<uchar>(i);
//                    for (int j = 0; j < roi.cols*3; j++) {
//                        if(roi.rows*roi.cols<1000){
//                            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x*3, i + rect.y), false) >= 0) {
//                                // if point is inside contour
//                                sum_r += data[j+2];
//                                sum_b += data[j];
//                            }
//                        }
//                    }
//
//                }
//                 Iterate through the ROI
                for (int i = 0; i < roi.rows; i++) {
                    for (int j = 0; j < roi.cols; j++) {
                        if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
                            // if point is inside contour
                            sum_r += roi.at<cv::Vec3b>(i, j)[2];
                            sum_b += roi.at<cv::Vec3b>(i, j)[0];
                        }
                    }
                }

                // Sum of red pixels > sum of blue pixels ?
                light.color = sum_r > sum_b ? RED : BLUE;
                lights.emplace_back(light);
            }

        }
    }
#ifdef SHOW_LIGHTS
    if(!lights.empty()){
        for(int i=0;i<lights.size();i++){
            rectangle(light_debug,lights[i].boundingRect().tl(),lights[i].boundingRect().br(),Scalar(20,0,255),2);
        }
        resize(light_debug,light_debug,Size(light_debug.size().width*1.6,light_debug.size().height*1.6));
        imshow("lights",light_debug);
    }
#endif


    return lights;
}

bool Detector::isLight(const Light & light)
{
    // The ratio of light (short side / long side)
    float ratio = light.length / light.width;
    bool ratio_ok = light_param.min_ratio < ratio && ratio < light_param.max_ratio;
    bool angle_ok = light.tilt_angle < light_param.max_angle;
#ifdef SHOW_LIGHTS
    string s_ratio = "r:" + to_string(int(ratio))+ "." +to_string(int(ratio * 100+0.5)%100);
    string s_angle = "a:" + to_string(int(light.tilt_angle))+ "." +to_string(int(light.tilt_angle * 100+0.5)%100);

    putText(light_debug,s_ratio,light.center+Point2f(5,0),1,0.8,Scalar(255,159,0),1);
    putText(light_debug,s_angle,light.center+Point2f(5,12),1,0.8,Scalar(0,255,0),1);
#endif
    bool is_light = ratio_ok && angle_ok;
    return is_light;
}


std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights) {
    std::vector<Armor> armors;

    // Loop all the pairing of lights
    for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
        for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
//            cout<<"detect:::"<<detect_color<<endl;
//            cout<<"ligt1"<<light_1->color<<endl;
            if (light_1->color != detect_color || light_2->color != detect_color){
                continue;
            }
#ifdef LOOSE_PROCESS
            if (containLight(*light_1, *light_2, lights)) {
                continue;
            }
#endif
            auto type = isArmor(*light_1, *light_2);
            if (type != ArmorType::INVALID) {
                auto armor = Armor(*light_1, *light_2);
                armor.armor_type = type;
                armors.emplace_back(armor);
            }


        }
    }
    if(!armors.empty()){
        for(int i=0;i<armors.size();i++){
            armors[i].pt[1] = armors[i].left_light.top;
            armors[i].pt[2] = armors[i].right_light.top;
            armors[i].pt[3] = armors[i].right_light.bottom;
            armors[i].pt[0] = armors[i].left_light.bottom;
//            circle(armor_debug,armors[i].left_light.top,6,Scalar(255,0,255),2);
//            circle(armor_debug,armors[i].right_light.bottom,6,Scalar(255,0,255),2);
#ifdef SHOW_ARMORS
            for(int j = 0; j<4 ;j++){
                line(armor_debug,armors[i].pt[j],armors[i].pt[(j+1)%4],Scalar(0,255,255),2,6);
            }
//            for(int j = 0; j<4 ;j++){
//                line(armor_debug,armors[i].pt[0],armors[i].pt[2],Scalar(0,255,255),2,6);
//                line(armor_debug,armors[i].pt[1],armors[i].pt[3],Scalar(0,255,255),2,6);
//            }
            for(int j = 0; j<4 ;j++){
                line(armor_debug,armors[i].pt[0],armors[i].pt[2],Scalar(0,255,255),2,6);
                line(armor_debug,armors[i].pt[1],armors[i].pt[3],Scalar(0,255,255),2,6);
            }
#endif
            Rect rect = Rect(armors[i].left_light.top,armors[i].right_light.bottom);
            armors[i].armor_rect = rect;
            // cout<<" ---- rect size: "<<rect.size()<<endl;
        }
#ifdef SHOW_ARMORS
        //        resize(debug1,debug1,Size(debug1.size().width*10,debug1.size().height*10));
//        imshow("111",debug1);
        resize(armor_debug,armor_debug,Size(armor_debug.size().width*1.6,armor_debug.size().height*1.6));
        imshow("armor debug", armor_debug);

#endif

    }
    return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool Detector::containLight(
        const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
    auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
    auto bounding_rect = cv::boundingRect(points);

    for (const auto & test_light : lights) {
        if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

        if (
            bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
            bounding_rect.contains(test_light.center)) {
            return true;
        }
    }
    return false;
}

ArmorType Detector::isArmor(const Light & light_1, const Light & light_2) const {

    // Ratio of the length of 2 lights (short side / long side)
    double light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                                : light_2.length / light_1.length;
    bool light_ratio_ok = light_length_ratio > armor_param.min_light_ratio;

    // Distance between the center of 2 lights (unit : light length)
    double avg_light_length = (light_1.length + light_2.length) / 2;
    double center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
    bool center_distance_ok = (armor_param.min_small_center_distance < center_distance &&
                               center_distance < armor_param.max_small_center_distance) ||
                              (armor_param.min_large_center_distance < center_distance &&
                               center_distance < armor_param.max_large_center_distance);

    // Angle of light center connection
    cv::Point2f diff = light_1.center - light_2.center;
    double angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
//    cout<<"angle:"<<angle<<endl;
    bool angle_ok = angle < armor_param.max_angle;
    angle_ok = 1;
    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

    // Judge armor type
    ArmorType type;
    if (is_armor) {
//        cout<<"q eee                  "<<armor_param.min_large_center_distance<<endl;
        type = center_distance > armor_param.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
    } else {
        type = ArmorType::INVALID;
    }
#ifdef SHOW_ARMORS
    string s_center_distance =
            "d:" + to_string(int(center_distance)) + "." + to_string(int(center_distance * 100 + 0.5) % 100);
    string s_light_length_ratio =
            "r:" + to_string(int(light_length_ratio)) + "." + to_string(int(light_length_ratio * 100 + 0.5) % 100);
    string s_angle = "a:" + to_string(int(angle)) + "." + to_string(int(angle * 100 + 0.5) % 100);
//
//    putText(armor_debug, s_center_distance, armor.center + Point2f(0, 20), 2, 0.5, Scalar(144, 255, 0), 1);
//    putText(armor_debug, s_light_length_ratio, armor.center + Point2f(0, 40), 2, 0.5, Scalar(0, 255, 144), 1);
//    putText(armor_debug, s_angle, armor.center + Point2f(0, 60), 2, 0.5, Scalar(144, 0, 255), 1);

#endif
    return type;
}

//bool Detector::isTracking(const Mat &img) {
//    Rect2d track_roi = track_rect; // the last goal_armor rect;
//    if (track_roi.size().area() == 0) {
//        return false;
//    }
//
//    Rect2d expand_track_roi;    // expand the last armor rect
//    expand_track_roi.x = track_roi.x - track_roi.width + relative_coord.x;
//    expand_track_roi.y = track_roi.y - track_roi.height * 2.2 + relative_coord.y;
//
//    expand_track_roi.width = track_roi.width * 2.8;
//    expand_track_roi.height = track_roi.height * 8.4;
//    expand_track_roi &= cv::Rect2d(0, 0, img.cols, img.rows);
//
//    if (expand_track_roi.height <= 0 || expand_track_roi.width <= 0) {
//        return false;
//    } else {
//        armorSrc = img(expand_track_roi).clone();
//        relative_coord = cv::Point2f(expand_track_roi.x, expand_track_roi.y);
//        return true;
//    }
//}


vector<Armor> Detector::trackArmor(Mat &img,vector<Armor> &armors) {
    armorSrc = img.clone();
//#ifdef TRACKER_ON
////    cout<<"              "<<track_flag<<endl;
//    if(track_flag == 1){
//        if(isTracking(armorSrc))
//            track_flag = 1;
//        else
//            track_flag = 0;
//    }
//#endif
//


    armors = matchLights(findLights(armorSrc,preprocessImage(armorSrc)));

    if(!armors.empty()){
        track_flag =1;
        track_rect = armors[0].armor_rect;
        nc.extractNumbers(armorSrc,armors);
        nc.doClassify(armors,more_strict_classification);
    }else{
        track_flag = 0;

    }

    for(int i =0;i<armors.size();i++){
        armors[i].center += relative_coord;
        armors[i].left_light.top += relative_coord;
        armors[i].right_light.top += relative_coord;
        armors[i].left_light.bottom += relative_coord;
        armors[i].right_light.bottom += relative_coord;
    }

    return armors;
}