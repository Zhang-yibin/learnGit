//
// Created by eski on 6/30/22.
//
/*

#include "classifier.h"

void NumberClassifier::extractNumbers(const cv::Mat & src, std::vector<Armor> & armors)
{

    debug = src.clone();
    // Light length in image
    const int light_length = 12;
    // Image size after warp
    const int warp_height = 28;
    const int small_armor_width = 32;
    const int large_armor_width = 54;
    // Number ROI size
    const cv::Size roi_size(20, 28);

    for (auto & armor : armors) {
        // Warp perspective transform
        cv::Point2f lights_vertices[4] = {
                armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
                armor.right_light.bottom};

        const int top_light_y = (warp_height - light_length) / 2 - 1;
        const int bottom_light_y = top_light_y + light_length;
        const int warp_width = armor.armor_type == SMALL ? small_armor_width : large_armor_width;
        cv::Point2f target_vertices[4] = {
                cv::Point(0, bottom_light_y),
                cv::Point(0, top_light_y),
                cv::Point(warp_width - 1, top_light_y),
                cv::Point(warp_width - 1, bottom_light_y),
        };
        cv::Mat number_image;

        auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
        cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));


        // Get ROI
        number_image =
                number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

        // Binarize
        cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
        cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        armor.number_img = number_image;

        imshow("qwq",number_image);
    }
}


// Weight 1 > 3 > 4 > 5 > 2
bool weightJudge(Armor armor1,Armor armor2) {
    if (armor1.number == '2')
        return false;
    else if (armor2.number == '2')
        return true;

    return armor1.number < armor2.number;
}

bool Cofidence_sort(Armor & a1, Armor & a2) {
    return  a1.confidence > a2.confidence;
}

void NumberClassifier::doClassify(std::vector<Armor> & armors,bool erase_n) {

    for (auto &armor : armors) {
        cv::Mat image = armor.number_img.clone();
        // Normalize
        image = image / 255.0;

        // Create blob from image
        cv::Mat blob;
        cv::dnn::blobFromImage(image, blob, 1., cv::Size(28, 20));

        // Set the input blob for the neural network
        net_.setInput(blob);

//cv::Mat input;
//input=image.reshape(0,1);
//net_.setInput(input);

        // Forward pass the image blob through the model
        cv::Mat outputs = net_.forward();

        // Do softmax
        float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::Mat softmax_prob;
        cv::exp(outputs - max_prob, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;

        double confidence;
        cv::Point class_id_point;
        minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int label_id = class_id_point.x;

        armor.confidence = confidence;
        armor.number = class_names_[label_id];

        std::stringstream result_ss;
        result_ss << armor.number << ":_" << std::fixed << std::setprecision(1)
                  << armor.confidence * 100.0 << "%";
        armor.classification_result = result_ss.str();
    }


    if(armors.size() >= 0) {
        sort(armors.begin(), armors.end(),
             [](Armor &target1, Armor &target2) {
                 return weightJudge(target1,target2);
             });
    }

    if(erase_n == 1) {
        armors.erase(
                std::remove_if(
                        armors.begin(), armors.end(),
                        [this](const Armor &armor) {
                            if (armor.confidence < threshold || armor.number == 'N') {
                                return true;
                            }

                            bool mismatch = false;
                            if (armor.armor_type == LARGE) {
                                mismatch = armor.number == 'O' || armor.number == '2' ||  armor.number == 'G';
                            } else if (armor.armor_type == SMALL) {
                                mismatch = armor.number == '1' || armor.number == 'B';
                            }
                            return mismatch;
                        }),
                armors.end());
    }
//    cp_armors.resize(armors.size());
//    copy(armors.begin(),armors.end(),cp_armors.begin());
//    armors.clear();
//    cout<<"armor_size:"<<armors.size()<<endl;
//    for (auto &armor : cp_armors) {
//         if(armor.confidence<threshold){
//             continue;
//         }else if(armor.number=='2'||armor.number=='O'){
//             continue;
//         }else{
//             armors.push_back(armor);
//         }
//    }
//
//    sort(armors.begin(), armors.end(), Cofidence_sort);
//
//
//
//    for(int i = 0; i < armors.capacity() ; i++) {
//        cout << "     i   : " <<i<<"      "<< armors[i].classification_result << endl;
//    }
//
//    cout << "result              " << armors[0].classification_result << endl;
//    cout<<"last size"<<armors.size()<<endl;

}
*/

// Copyright 2022 Chen Jun
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "classifier.h"
//namespace rm_auto_aim
//{
/*    NumberClassifier::NumberClassifier(
            const std::string & model_path, const std::string & label_path, const double thre,
            const std::vector<std::string> & ignore_classes)
            : threshold(thre), ignore_classes_(ignore_classes)
    {
        net_ = cv::dnn::readNetFromONNX(model_path);

        std::ifstream label_file(label_path);
        std::string line;
        while (std::getline(label_file, line)) {
            class_names_.push_back(line);
        }
    }*/

    void NumberClassifier::extractNumbers(const cv::Mat & src, std::vector<Armor> & armors)
    {
        // Light length in image
        const int light_length = 12;
        // Image size after warp
        const int warp_height = 28;
        const int small_armor_width = 32;
        const int large_armor_width = 54;
        // Number ROI size
        const cv::Size roi_size(20, 28);

        for (auto & armor : armors) {
            // Warp perspective transform
            cv::Point2f lights_vertices[4] = {
                    armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
                    armor.right_light.bottom};

            const int top_light_y = (warp_height - light_length) / 2 - 1;
            const int bottom_light_y = top_light_y + light_length;
            const int warp_width = armor.armor_type == SMALL ? small_armor_width : large_armor_width;
            cv::Point2f target_vertices[4] = {
                    cv::Point(0, bottom_light_y),
                    cv::Point(0, top_light_y),
                    cv::Point(warp_width - 1, top_light_y),
                    cv::Point(warp_width - 1, bottom_light_y),
            };
            cv::Mat number_image;
            auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
            cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

            // Get ROI
            number_image =
                    number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

            // Binarize
            cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
            cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

            armor.number_img = number_image;
            imshow("qwq",number_image);
        }
    }

    void NumberClassifier::doClassify(std::vector<Armor> &armors, bool erase_n)
    {
        for (auto & armor : armors) {
            cv::Mat image = armor.number_img.clone();

            // Normalize
            image = image / 255.0;

            // Create blob from image
            cv::Mat blob;
            cv::dnn::blobFromImage(image, blob);

            // Set the input blob for the neural network
            net_.setInput(blob);
            // Forward pass the image blob through the model
            cv::Mat outputs = net_.forward();

            // Do softmax
            float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
            cv::Mat softmax_prob;
            cv::exp(outputs - max_prob, softmax_prob);
            float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
            softmax_prob /= sum;

            double confidence;
            cv::Point class_id_point;
            minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
            int label_id = class_id_point.x;

            armor.confidence = confidence;
            armor.number = class_names_[label_id];

            std::stringstream result_ss;
            result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
                      << armor.confidence * 100.0 << "%";
            armor.classification_result = result_ss.str();
        }
        armors.erase(
                std::remove_if(
                        armors.begin(), armors.end(),
                        [this](const Armor & armor) {
                            if (armor.confidence < threshold || armor.number == 'N') {
                                return true;
                            }

                            bool mismatch_armor_type = false;
                            if (armor.armor_type == LARGE) {
                                mismatch_armor_type =
                                        armor.number == 'O' || armor.number == '2' || armor.number == 'G';
                            } else if (armor.armor_type == SMALL) {
                                mismatch_armor_type = armor.number == '1' || armor.number == 'B';
                            }
                            return mismatch_armor_type;
                        }),
                armors.end());
    }

//}  // namespace rm_auto_aim