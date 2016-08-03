/***********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, fengkaijie, myyerrol
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
 *  * Neither the name of the fengkaijie, myyerrol nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 *  fengkaijie       2016.7.20     Create this file
 *  myyerrol         2016.8.1      Modify the format
 *
 *  Description:
 *  This .cpp file implements face recognition class.
 **********************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <vector>
#include <string>
#include "face_recognition.h"

std::string g_name_cascade = "../face_detection/data/haarcascades/haarcascade_frontalface_alt.xml";
std::string g_name_nested_cascade = "../face_detection/data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
std::string g_save_model_path = "/home/ubuntu/Desktop/woodbox_mobile_car/jetson_tk1/face_recognition/face_recognition_model.txt";
cv::Ptr<cv::FaceRecognizer> g_model;
FaceRecognition g_face_recognition;

FaceRecognition::FaceRecognition(void)
{
    serial_id_ = open(DEVICE, O_WRONLY);

    if (!serial_id_) {
        std::cout << "Open UART failed!" << std::endl;
        close(serial_id_);
        exit(1);
    }
}

FaceRecognition::~FaceRecognition(void)
{
    close(serial_id_);
}

void FaceRecognition::detectAndDraw(cv::Mat image,
                                    cv::CascadeClassifier &cascade,
                                    cv::CascadeClassifier &nested_cascade,
                                    double scale,
                                    bool try_flip)
{
    int    i    = 0;
    double tick = 0.0;
    std::vector<cv::Rect> faces_a, faces_b;
    cv::Scalar colors[] = {
        CV_RGB(0, 0, 255),
        CV_RGB(0, 128, 255),
        CV_RGB(0, 255, 255),
        CV_RGB(0, 255, 0),
        CV_RGB(255, 128, 0),
        CV_RGB(255, 255, 0),
        CV_RGB(255, 0, 0),
        CV_RGB(255, 0, 255)
    };
    cv::Mat image_gray;
    cv::Mat image_small(cvRound(image.rows / scale),
                        cvRound(image.cols / scale),
                        CV_8UC1);
    // Convert to gray image.
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    // Convert gray image to small size.
    cv::resize(image_gray, image_small, image_small.size(), 0, 0,
               cv::INTER_LINEAR);
    cv::equalizeHist(image_small, image_small);

    tick = (double)cvGetTickCount();
    cascade.detectMultiScale(image_small, faces_a, 1.1, 2, 0 |
                             CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

    if (try_flip) {
        cv::flip(image_small, image_small, 1);
        cascade.detectMultiScale(image_small, faces_b, 1.1, 2, 0 |
                                 CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
        std::vector<cv::Rect>::const_iterator it = faces_b.begin();
        for (; it != faces_b.end(); it++) {
            faces_a.push_back(cv::Rect(image_small.cols - it->x - it->width,
                                       it->y, it->width, it->height));
        }
    }

    // Calculate detection's time.
    tick = (double)cvGetTickCount() - tick;
    std::cout << "Detection time: "
              << tick / ((double)cvGetTickCount() * 1000.0)
              << " ms"
              << std::endl;

    std::vector<cv::Rect>::const_iterator it = faces_a.begin();
    for (; it != faces_a.end(); it++, i++) {
        int radius;
        double aspect_ratio = (double)it->width / it->height;
        std::vector<cv::Rect> nested_objects;
        cv::Mat small_image_roi;
        cv::Point center;
        cv::Scalar color = colors[i % 8];

        // Capture detected face and predict it.
        cv::Mat image_gray;
        cv::Mat image_result(cvRound(IMG_HEIGH), cvRound(IMG_WIDTH), CV_8UC1);
        cv::Mat image_temp;
        cv::Rect rect;
        rect.x = cvRound(it->x * scale);
        rect.y = cvRound(it->y * scale);
        rect.height = cvRound(it->height * scale);
        rect.width  = cvRound(it->width  * scale);
        image_temp  = image(rect);
        cv::cvtColor(image_temp, image_gray, CV_BGR2GRAY);
        cv::resize(image_gray, image_result, image_result.size(), 0, 0,
                   cv::INTER_LINEAR);
        int predicted_label = g_model->predict(image_result);

        std::cout << "*************************" << std::endl
                  << " The predicted label: "    << predicted_label
                  << std::endl
                  << "*************************"
                  << std::endl;

        // Recognize specific face for sending character to serial device.
        if (predicted_label == 1) {
            g_face_recognition.writeCharToSerial('Y');
        }
        else {
            g_face_recognition.writeCharToSerial('N');
        }

        // Draw the circle for faces.
        if (0.75 < aspect_ratio && aspect_ratio > 1.3) {
            center.x = cvRound((it->x + it->width * 0.5) * scale);
            center.y = cvRound((it->y + it->height * 0.5) * scale);
            radius = cvRound((it->width + it->height) * 0.25 * scale);
            cv::circle(image, center, radius, color, 3, 8, 0);
        }
        else {
            // Draw the rectangle for faces.
            cv::rectangle(image,
                          cvPoint(cvRound(it->x * scale),
                                  cvRound(it->y * scale)),
                          cvPoint(cvRound((it->x + it->width  - 1) * scale),
                                  cvRound((it->y + it->height - 1) * scale)),
                          color,
                          3,
                          8,
                          0);
            if (nested_cascade.empty()) {
                continue ;
            }
            small_image_roi = image_small(*it);
            nested_cascade.detectMultiScale(small_image_roi, nested_objects,
                                            1.1, 2, 0 | CV_HAAR_SCALE_IMAGE,
                                            cv::Size(30, 30));
            std::vector<cv::Rect>::const_iterator it_temp =
                nested_objects.begin();
            // Draw the circle for eyes.
            for (; it_temp != nested_objects.end(); it_temp++) {
                center.x = cvRound((it->x + it_temp->x + it_temp->width * 0.5)
                    * scale);
                center.y = cvRound((it->y + it_temp->y + it_temp->height * 0.5)
                    * scale);
                radius = cvRound((it_temp->width + it_temp->height) * 0.25
                    * scale);
                cv::circle(image, center, radius, color, 3, 8, 0);
            }
        }
    }
    // Open camera window.
    cv::imshow("Face Recognition", image);
}

void FaceRecognition::getUseHelp(void)
{
    std::cout << std::endl
              << "This program demonstrates the cascade recognizer. Now you "
              << "can use Haar or LBP features.\n"
              << "This classifier can recognize many kinds of rigid objects, "
              << "once the appropriate classifier is trained.\n"
              << "It's most known use is for faces.\n"
              << "Usage:\n"
              << "./face_recognition\n"
              << "[--cascade=<cascade_path> this is the primary trained "
              << "classifier such as frontal face]\n"
              << "[--nested-cascade[=nested_cascade_path this an optional "
              << "secondary classifier such as eyes]]\n"
              << "[--scale=<image scale greater or equal to 1, try 1.3 for "
              << "example>]\n"
              << "[--try-flip]\n"
              << "[filename|camera_index]\n\n"
              << "During execution:\n\tHit any key to quit.\n"
              << "\tUsing OpenCV version "
              << CV_VERSION
              << std::endl
              << std::endl;
}

bool FaceRecognition::writeCharToSerial(char ch)
{
    if (write(serial_id_, &ch, sizeof(ch)) == -1) {
        return false;
    }

    return true;
}

int main(int argc, char **argv)
{
    bool try_flip = false;
    std::string scale_param = "--scale=";
    size_t scale_param_length = scale_param.length();
    std::string cascade_param = "--cascade=";
    size_t cascade_param_length = cascade_param.length();
    std::string nested_cascade_param = "--nested-cascade=";
    size_t nested_cascade_param_length =  nested_cascade_param.length();
    std::string try_flip_param = "--try-flip=";
    size_t try_flip_param_length = try_flip_param.length();
    std::string name_input;
    double scale = 1.0;
    cv::Mat frame;
    cv::Mat frame_copy;
    cv::Mat image;
    cv::CascadeClassifier cascade;
    cv::CascadeClassifier nested_cascade;
    CvCapture *capture = 0;

    g_face_recognition.getUseHelp();

    // Load trained model.
    g_model = cv::createLBPHFaceRecognizer();
    g_model->load(g_save_model_path);

    std::cout << "*************************" << std::endl
              << "   Load model finished   " << std::endl
              << "*************************" << std::endl
              << std::endl;

    // Load command line parameters.
    for (int i = 1; i < argc; i++) {
        std::cout << "Processing " << i << " " << argv[i] << std::endl;
        if (cascade_param.compare(0,
                                  cascade_param_length,
                                  argv[i],
                                  cascade_param_length) == 0) {
            g_name_cascade.assign(argv[i] + cascade_param_length);
            std::cout << "Cascade name: " << g_name_cascade
                      << std::endl;
        }
        else if (nested_cascade_param.compare(0,
                                              nested_cascade_param_length,
                                              argv[i],
                                              nested_cascade_param_length) == 0) {
            if (argv[i][nested_cascade_param.length()] == '=') {
               g_name_nested_cascade.assign(argv[i] +
                                            nested_cascade_param.length() + 1);
            }
            if (!nested_cascade.load(g_name_nested_cascade)) {
                std::cerr << "Load nested cascade classifier failed!"
                          << std::endl;
            }
        }
        else if (scale_param.compare(0,
                                     scale_param_length,
                                     argv[i],
                                     scale_param_length) == 0) {
            if(!sscanf(argv[i] + scale_param.length(), "%lf", &scale) ||
                scale < 1) {
                scale = 1;
            }
            std::cout << "Scale: " << scale << std::endl;
        }
        else if (try_flip_param.compare(0,
                                        try_flip_param_length,
                                        argv[i],
                                        try_flip_param_length) == 0) {
            try_flip = true;
            std::cout << " Try to flip image horizontally."
                      << std::endl;
        }
        else if (argv[i][0] == '-') {
            std::cerr << "Unknown option %s" << argv[i] << std::endl;
        }
        else {
            name_input.assign(argv[i]);
        }
    }

    if (!cascade.load(g_name_cascade)) {
        std::cerr << "Load cascade classifier failed!" << std::endl;
        g_face_recognition.getUseHelp();
        return -1;
    }

    if (name_input.empty() || (isdigit(name_input.c_str()[0]) &&
        name_input.c_str()[1]) == '\0') {
        capture = cvCaptureFromCAM(name_input.empty() ? 0 :
                                   name_input.c_str()[0] - '0');
        int camera = name_input.empty() ? 0 : name_input.c_str()[0] - '0';
        if (!capture) {
            std::cerr << "Capture from CAM " << camera << "doesn't work!"
                      << std::endl;
        }
    }
    else if (name_input.size()) {
        image = cv::imread(name_input, 1);
        if (image.empty()) {
            capture = cvCaptureFromAVI(name_input.c_str());
            if (!capture) {
                std::cerr << "Capture from AVI doesn't work!" << std::endl;
            }
        }
    }
    else {
        image = cv::imread("lena.jpg", 1);
        if (image.empty()) {
            std::cerr << "Read lena.jpg failed!" << std::endl;
        }
    }

    cvNamedWindow("Face Recognition", 1);

    if (capture) {
        std::cout << "In capture..." << std::endl;
        for (;;) {
            IplImage *ipl_image = cvQueryFrame(capture);
            frame = ipl_image;
            while (frame.dims == 0) {
                ipl_image = cvQueryFrame(capture);
                frame = ipl_image;
            }
            if (frame.empty()) {
                break;
            }
            if (ipl_image->origin == IPL_ORIGIN_TL) {
                frame.copyTo(frame_copy);
            }
            else {
                cv::flip(frame, frame_copy, 0);
            }

            g_face_recognition.detectAndDraw(frame_copy,
                                             cascade,
                                             nested_cascade,
                                             scale,
                                             try_flip);

            // Press any keys to close the camera and exit.
            if (cv::waitKey(10) > 0) {
                goto cleanup;
            }
        }
        cv::waitKey(0);
cleanup:
        cvReleaseCapture(&capture);
    }
    else {
        std::cout << "In image read..." << std::endl;
        if (!image.empty()) {
            g_face_recognition.detectAndDraw(image,
                                             cascade,
                                             nested_cascade,
                                             scale,
                                             try_flip);
            cv::waitKey(0);
        }
        else if (!name_input.empty()) {
            FILE *file = fopen(name_input.c_str(), "rt");
            if (file) {
                char buffer[1000 + 1];
                while (fgets(buffer, 1000, file)) {
                    int length = (int)strlen(buffer);
                    int key = 0;
                    while (length > 0 && isspace(buffer[length - 1])) {
                        length--;
                    }
                    buffer[length] = '\0';
                    std::cout << "File " << buffer << std::endl;
                    image = cv::imread(buffer, 1);
                    if (!image.empty()) {
                        g_face_recognition.detectAndDraw(image,
                                                         cascade,
                                                         nested_cascade,
                                                         scale,
                                                         try_flip);
                        key = cv::waitKey(0);
                        if (key == 27 || key == 'q' || key == 'Q') {
                            break;
                        }
                    }
                    else {
                        std::cerr << "Read image failed!" << std::endl;
                    }
                }
                fclose(file);
            }
        }
    }
    cvDestroyWindow("Face Detection");
    return 0;
}
