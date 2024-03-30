//
// Created by eski on 6/30/22.
//

#ifndef SR_SDUST_CLASSIFIER_H
#define SR_SDUST_CLASSIFIER_H

#include <fstream>
#include "detector.h"

class NumberClassifier
{
public:
    NumberClassifier(
            const std::string & model_path, const std::string & label_path, const double thre)
            : threshold(thre) {
        net_ = cv::dnn::readNetFromONNX(model_path);

        std::ifstream label_file(label_path);
        std::string line;
        while (std::getline(label_file, line)) {
            class_names_.push_back(line[0]);
        }
    }
    void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);
    void doClassify(std::vector<Armor> & armors, bool erase_n);
    double threshold = 70;
    Mat debug;
    std::vector<Armor> cp_armors;

private:
    cv::dnn::Net net_;
    std::vector<char> class_names_;
    std::vector<std::string> ignore_classes_;
};



#endif //SR_SDUST_CLASSIFIER_H
