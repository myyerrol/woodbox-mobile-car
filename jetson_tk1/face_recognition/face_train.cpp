/***********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Philipp Wagner, myyerrol
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of the Philipp Wagner, myyerrol nor the names of
 *    its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ***********************************************************************

 ***********************************************************************
 *  History:
 *  <Authors>        <Date>        <Operation>
 *  Philipp Wagner   2011          Create this file
 *  myyerrol         2016.8.1      Modify the format
 *
 *  Description:
 *  This .cpp file implements face train class.
 **********************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
#include "face_train.h"

FaceTrain::FaceTrain(void)
{
}

FaceTrain::~FaceTrain(void)
{
}

void FaceTrain::readCSVFile(std::string &file_name,
                            std::vector<cv::Mat> &images,
                            std::vector<int> &labels,
                            std::map<int, std::string> &labels_info,
                            char separator)
{
    std::ifstream file_csv(file_name.c_str());

    if (!file_csv) {
        CV_Error(CV_StsBadArg, "Input file is not valid!");
    }

    std::string line;
    std::string path;
    std::string classlabel;
    std::string info;

    while (std::getline(file_csv, line)) {
        std::stringstream string_lines(line);
        path.clear();
        classlabel.clear();
        info.clear();
        std::getline(string_lines, path, separator);
        std::getline(string_lines, classlabel, separator);
        std::getline(string_lines, info, separator);
        if (!path.empty() && !classlabel.empty()) {
            std::cout << "Processing " << path << std::endl;
            int label = atoi(classlabel.c_str());
            if (!info.empty()) {
                labels_info.insert(std::make_pair(label, info));
            }
            cv::String root(path.c_str());
            std::vector<cv::String> files;
            cv::glob(root, files, true);
            std::vector<cv::String>::const_iterator f = files.begin();
            for (; f != files.end(); ++f) {
                std::cout << "\t" << *f << std::endl;
                cv::Mat image = cv::imread(*f, CV_LOAD_IMAGE_GRAYSCALE);
                int width  = -1;
                int height = -1;
                bool warning = true;
                if (width > 0 && height > 0 &&
                   (width != image.cols || height != image.rows)) {
                    std::cout << "Images should be of the same size!"
                              << std::endl;
                }
                if (warning && (image.cols < 50 || image.rows < 50)) {
                    std::cout << "Images should be not smaller than 50x50!"
                              << std::endl;
                    warning = false;
                }
                images.push_back(image);
                labels.push_back(label);
            }
        }
    }
}

void FaceTrain::getUseHelp(char *argv)
{
    std::cout << "Usage: \n"
              << argv
              << " "
              << "[csv] [arg]\n"
              << "[csv]: path to config file in csv format.\n"
              << "[arg]: if the 2nd argument is provided (with any value), "
              << "the advanced stuff is run and shown to console.\n\n"
              << "The [csv] config file consists of the following lines:\n"
              << "<path>;<label>[;<comment>]\n"
              << std::endl;
}

int main(int argc, char **argv)
{
    FaceTrain face_train;

    // Check for valid command line arguments.
    if (argc != 2 && argc != 3) {
        face_train.getUseHelp(argv[0]);
        exit(1);
    }

    // Get the path to your csv file.
    std::string file_csv = std::string(argv[1]);
    // These vectors hold the images and corresponding labels.
    std::vector<cv::Mat> images;
    std::vector<int>     labels;
    std::map<int, std::string> labels_info;

    // Read in the data. This can fail if no valid input filename is given.
    try {
        face_train.readCSVFile(file_csv, images, labels, labels_info);
    }
    catch (cv::Exception &ex) {
        std::cerr << "Open file \"" << file_csv << "\" failed! Reason: "
                  << ex.msg << std::endl;
        exit(1);
    }

    // Quit if there are not enough images for this demo.
    if (images.size() <= 1) {
        std::string message_a = "This program needs at least two images ";
        std::string message_b = "to work. Add more images to your dataset!";
        std::string error_message = message_a + message_b;
        CV_Error(CV_StsError, error_message);
    }

    // The following lines simply get the last images from your dataset and
    // remove it from the vector. This is done, so that the training data
    // (which we learn the cv::FaceRecognizer on) and the test data we test the
    // model with, do not overlap.
    cv::Mat test_sample = images[images.size() - 1];
    int test_label = labels[labels.size() - 1];
    images.pop_back();
    labels.pop_back();

    // The following lines create an Eigenfaces model for face recognition and
    // train it with the images and labels read from the given csv file.
    cv::Ptr<cv::FaceRecognizer> model = cv::createEigenFaceRecognizer();
    model->setLabelsInfo(labels_info);
    model->train(images, labels);
    std::string save_model_path = "face_recognition_model.txt";
    std::cout << "Save the trained model to " << save_model_path << std::endl;
    model->save(save_model_path);

    // The following line predicts the label of a given test image.
    int predicted_label = model->predict(test_sample);
    std::string result_message = cv::format("Predicted: %d / Actual : %d.",
                                             predicted_label, test_label);
    std::cout << result_message << std::endl;

    if ((predicted_label == test_label) &&
        !model->getLabelInfo(predicted_label).empty()) {
        std::cout << cv::format("%d-th label's info: %s", predicted_label,
                                 model->getLabelInfo(predicted_label).c_str())
                  << std::endl;
    }

    if (argc > 2) {
        // Sometimes you'll need to get/set internal model data, which isn't
        // exposed by the public cv::FaceRecognizer. Since each
        // cv::FaceRecognizer is derived from a cv::Algorithm, you can query
        // the data.
        // First we'll use it to set the threshold of the FaceRecognizer to 0.0
        // without retraining the model. This can be useful if you are
        // evaluating the model.
        model->set("threshold", 0.0);
        // Now the threshold of this model is set to 0.0. A prediction now
        // returns -1, as it's impossible to have a distance below it.
        predicted_label = model->predict(test_sample);
        std::cout << "Predicted class: " << predicted_label << std::endl;
        // Here is how to get the eigenvalues of this Eigenfaces model.
        cv::Mat eigen_values = model->getMat("eigenvalues");
         // And we can do the same to display the Eigenvectors.
        cv::Mat eigen_vectors = model->getMat("eigenvectors");
        // From this we will display the (at most) first 10 Eigenfaces.
        for (int i = 0; i < std::min(10, eigen_vectors.cols); i++) {
            std::string message = cv::format("Eigenvalue #%d: %.5f", i,
                                              eigen_values.at<double>(i));
            std::cout << message << std::endl;
            cv::Mat eigen_vector = eigen_vectors.col(i).clone();
            // Reshape to original size & normalize to [0...255] for imshow.
            cv::Mat gray_scale;
            cv::normalize(eigen_vector.reshape(1), gray_scale, 0, 255,
                          cv::NORM_MINMAX, CV_8UC1);
             // Show the image & apply a Jet colormap for better sensing.
            cv::Mat gray_scale_copy;
            cv::applyColorMap(gray_scale, gray_scale_copy, cv::COLORMAP_JET);
            cv::imshow(cv::format("%d", i), gray_scale_copy);
        }
        cv::waitKey(0);
    }
    return 0;
}
