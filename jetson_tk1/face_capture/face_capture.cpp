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
 *  This .cpp file implements face capture class.
 **********************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <string>
#include "face_capture.h"

FaceCapture::FaceCapture(void)
{
}

FaceCapture::~FaceCapture(void)
{
}

void FaceCapture::takePicture(cv::Mat image, int index)
{
    cv::Mat image_gray;
    cv::Mat image_goal(cvRound(IMAGE_HEIGHT), cvRound(IMAGE_WIDTH), CV_8UC1);
    std::stringstream temp;
    std::string name_index;

    temp << index;
    temp >> name_index;
    std::string name_image = name_index + ".bmp";

    // Convert to gray image.
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    // Convert gray image to default size.
    cv::resize(image_gray, image_goal, image_goal.size(), 0, 0,
               cv::INTER_LINEAR);
    // Save image.
    cv::imwrite(name_image, image_goal);
}

int main(int argc, char **argv)
{
    int number = 0;
    int number_index = 0;

    std::string number_param = "--number=";
    size_t number_param_length = number_param.length();

    cv::Mat image;
    CvCapture *capture = 0;

    FaceCapture face_capture;

    if (argc == 1) {
        number = 1;
        number_index = 1;
        std::cout << "The images number is one." << std::endl;
    }
    else {
        for (int i = 1; i < argc; i++) {
            if(number_param.compare(0, number_param_length, argv[i],
                                    number_param_length) == 0) {
                sscanf(argv[i] + number_param_length, "%d", &number);
                number_index = 1;
                std::cout << "The images number is " << number << "."
                          << std::endl;
            }
        }
    }

    // Open the camera.
    capture = cvCaptureFromCAM(CAMERA_INDEX);

    if(!capture) {
        std::cerr << "Camera doesn't work!" << std::endl;
        exit(1);
    }
    else {
        std::cout << "In capture..." << std::endl;
        std::cout << "Please press key on the keyboard to capture your face."
                  << std::endl;
        IplImage *ipl_image;
        while(true) {
            while(!(ipl_image = cvQueryFrame(capture))) {
                ;
            }
            image = ipl_image;
            cv::imshow("Face Capture", image);
            // Wait for key is pressed.
            if(cv::waitKey(10) >= 0) {
                if(number_index <= number) {
                    usleep(300);
                    face_capture.takePicture(image, number_index);
                    number_index++;
                }
                else {
                    std::cout << "Finish capture!" << std::endl;
                    break;
                }
            }
        }
        // Close camera.
        cvReleaseCapture(&capture);
        return 0;
    }
}
