// load the dynamic_reconfigure module: rosrun rqt_gui rqt_gui -s reconfigure


// load the dynamic_reconfigure module: rosrun rqt_gui rqt_gui -s reconfigure


#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <camera_publish/TutorialsConfig.h>

#include <algorithm>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include "opencv2/opencv.hpp"
// #include "CamObj.hpp"
#include <string>


#include <sys/stat.h> 
#include <sys/types.h> 
#include <time.h>	

#include <thread>



#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
// #include "popt_pp.h"
#include <stdlib.h>
#include <unistd.h>
#include "tcamimage.h"


#include <geometry_msgs/Point.h>
// #include <geometry_msgs/TransformStamped.h>

#include "camera_publish/my_msg.h"

#include <vector>
#include <fstream>

using namespace gsttcam;
using namespace std;
using namespace cv;
using namespace ros;
bool use_img = false;
int count_img(1);
int cam_index(1), cam_index2(0), old_exposure_time(5000), exposure_time(5000), cam_count(0), cam_count2(0) /*w(1440), h(1080)*/, x(0), skippedImg(1);
// cv::Mat img1, img2;
// HANDLE xiH = NULL;
// HANDLE xiH2 = NULL;
bool use_old_folder_flag(false), create_folder_flag(false), create_folder_flag_old(false), enable_use_input_folder_name(false);
string folder_name, input_folder_name;

// void findRectificationMap(FileStorage& calib_file, Size finalSize);
// void stereo_rec( cv::Mat & tmpL, cv::Mat & tmpR, cv::Mat & frame_rec_left, cv::Mat & frame_rec_right);

const std::string WHITESPACE = " \n\r\t\f\v";

FileStorage calib_file;
Mat XR, XT, Q, P1, P2, XQ;
Mat R1, R2, K1, K2, D1, D2, R;
Mat lmapx, lmapy, rmapx, rmapy;

Vec3d T;
Size calib_img_size, out_img_size;

// int x = 1440, y = 1080;
double res_div = 1;


std::string ltrim(const std::string& s)
{
    size_t start = s.find_first_not_of(WHITESPACE);
    return (start == std::string::npos) ? "" : s.substr(start);
}

std::string rtrim(const std::string& s)
{
    size_t end = s.find_last_not_of(WHITESPACE);
    return (end == std::string::npos) ? "" : s.substr(0, end + 1);
}


std::string trim(const std::string& s)
{
    return rtrim(ltrim(s));
}


void callback(camera_publish::TutorialsConfig &config, uint32_t level) {
    
    exposure_time = config.exposureTime;
    enable_use_input_folder_name = config.use_input_folder_name;
    input_folder_name = config.folder_name.c_str();
    create_folder_flag = config.create_new_folder_and_save_image;
    
    use_old_folder_flag = config.use_old_folder_and_save_image;
    skippedImg = config.skippedImage;
    if (create_folder_flag)
        cout << "create folder is true" << endl;
    else
        cout << "create folder is false" << endl;
}




void findRectificationMap(FileStorage& calib_file, Size finalSize) {
    Rect validRoi[2];
    stereoRectify(K1, D1, K2, D2, calib_img_size, R, Mat(T), R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, finalSize, &validRoi[0], &validRoi[1]);
    cv::initUndistortRectifyMap(K1, D1, R1, P1, finalSize, CV_32F, lmapx, lmapy);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, finalSize, CV_32F, rmapx, rmapy);
}



int w = 1440, h = 1080;




int main(int argc, char** argv)
{
    
    
    // read calibration information 
    string filename = "src/config/cam_stereo.yml";
    
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    
    if (!fs.isOpened()) {
        cerr << "Failed to open " << filename << endl;
        return 1;
    }
    
    fs["T"] >> T;
    fs["K1"] >> K1;
    fs["K2"] >> K2;
    fs["D1"] >> D1;
    fs["D2"] >> D2;
    fs["R"] >> R;
    
    XR = (Mat_<double>(3, 3) << 0.00080, 0.00080, 1.00000,
          -1.00000, 0.00000, 0.00080,
          0.00000, -1.00000, 0.00080);
    XT = (Mat_<double>(3, 1) << -4.0, 0.0, 1.7);
    
    
    
    
    if (mkdir("data", 0777) == -1) 
        cerr << "Error :  " << strerror(errno) << endl; 
    else
        cout << "data directory created\n";      
    
    
    gst_init(&argc, &argv);
    // Declaration of the Pointers to the properties.
    //     std::shared_ptr<Property> ExposureAuto = NULL;
    //     std::shared_ptr<Property> ExposureValue = NULL;
    //     std::shared_ptr<Property> GainAuto = NULL;
    //     std::shared_ptr<Property> GainValue = NULL;
    //     cv::Mat OpenCVImage, OpenCVImage2;
    // Initialize our TcamCamera object "cam" with the serial number
    // of the camera, which is to be used in this program.
    
    // my camera is here 03020388 and 03020395
    TcamImage cam("03020388"); 
    TcamImage cam2("03020395"); 
//     TcamImage cam2("14910439"); 
//     TcamImage cam("14910441"); 
    
    if(!use_img)
    {
        
        //     TcamImage cam2("14910439"); 
        //     TcamImage cam("14910441"); 
        cam.set_capture_format("BGRx", FrameSize{1440,1080}, FrameRate{30,1});
        
        cam2.set_capture_format("BGRx", FrameSize{1440,1080}, FrameRate{30,1});
        //     cam.enable_video_display(gst_element_factory_make("ximagesink", NULL));
        //     cam2.enable_video_display(gst_element_factory_make("ximagesink", NULL));
        
        // Start the camera
        cam.start();
        cam2.start();
        cam.set_auto_exposure();
        cam.set_exposure_time(exposure_time);
        cam2.set_auto_exposure();
        cam2.set_exposure_time(exposure_time);
        
        
       
        
        
        
        
        std::shared_ptr<Property> GainValue = NULL;
      
        
        try
        {
            GainValue = cam.get_property("Gain");
        }
        catch(std::exception &ex)    
        {
            printf("Error %s : %s\n",ex.what(), "Gain Value");
        }
        
          if( GainValue != NULL){
              cout <<"I am setting gain for cam " << endl;
            GainValue->set(cam,150);
        }
        
        
        std::shared_ptr<Property> GainValue2 = NULL;
      
        
        try
        {
            GainValue2 = cam2.get_property("Gain");
        }
        catch(std::exception &ex)    
        {
            printf("Error %s : %s\n",ex.what(), "Gain Value");
        }
        
          if( GainValue2 != NULL){
              cout <<"I am setting gain for cam2 " << endl;
            GainValue2->set(cam2,150);
        }
        
        
         std::shared_ptr<Property> GainAuto = NULL;


            GainAuto = cam.get_property("Gain Auto");
      
        
        
        
        
        if( GainAuto != NULL)
        {
            int Auto;
            GainAuto->get(cam,Auto);
            if( Auto == 1)
                printf("Current gain automatic is On.\n");
            else
                printf("Current gain  automatic is Off.\n");
        }
        
        
         if( GainValue != NULL)
    {
        int gain;
        GainValue->get(cam,gain);
        printf("Current gain value is %d.\n",gain);
    }
    
         if( GainValue2 != NULL)
    {
        int gain2;
        GainValue2->get(cam2,gain2);
        printf("Current gain2 value is %d.\n",gain2);
    }
        
        sleep(1);
    }
    
      
        

    
    //     VideoCapture cap1(1);
    //     VideoCapture cap2(0);
    //     cap1.set(CV_CAP_PROP_FRAME_WIDTH, w);
    //     cap1.set(CV_CAP_PROP_FRAME_HEIGHT, h);
    //     cap2.set(CV_CAP_PROP_FRAME_WIDTH, w);
    //     cap2.set(CV_CAP_PROP_FRAME_HEIGHT, h);
    //     
    
    init(argc, argv, "ImagingSource");
    NodeHandle nh;
    
    
    ros::Publisher pub = nh.advertise<camera_publish::my_msg>("my_topic", 1);
    // the message to be published
    camera_publish::my_msg msg;
    msg.another_field = 0;
    
    
    
    
    
    
    
    
    
    string camera_name_left("kitti_stereo/left"), camera_name_right("kitti_stereo/right");
    image_transport::ImageTransport it(nh);
    cv_bridge::CvImage raw_out_msg;
    
    image_transport::Publisher pub_cam_msg = it.advertise(camera_name_left+"/image_rect", 1);
    image_transport::Publisher pub_cam_msg_right = it.advertise(camera_name_right+"/image_rect", 1);
    
    Publisher pub_cam_info_left = nh.advertise<sensor_msgs::CameraInfo>(camera_name_left+"/camera_info", 1);
    
    sensor_msgs::ImagePtr cam_msg;
    const string camurl_left = "file:///src/cam_stereo_online_left.yml";
    camera_info_manager::CameraInfoManager caminfo_left(nh, camera_name_left, camurl_left);
    sensor_msgs::CameraInfo ci_left;
    
    cv_bridge::CvImage raw_out_msg_right;
    
    Rate loop_rate(10);
    Mat frame, frame2;    
    
    dynamic_reconfigure::Server<camera_publish::TutorialsConfig> server;
    dynamic_reconfigure::Server<camera_publish::TutorialsConfig>::CallbackType f;
    
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
    
    int index(0);
    int count_index(0);
    int count = 0;
    
    while (nh.ok()) 
    {
        
        msg.points.clear();
        
        msg.another_field = count;
//         int i = 0;
        
        
        
        if (exposure_time != old_exposure_time)
        {
            old_exposure_time = exposure_time;
            cout << "change the exposure time" << endl;
            
            if(!use_img)
            {
                
                cam.set_exposure_time(exposure_time);
                cam2.set_exposure_time(exposure_time);
            }
        }
        
        
        auto now_time = std::chrono::system_clock::now();
        std::time_t nowTime = std::chrono::system_clock::to_time_t(now_time);
        
        string folder_name_temp = trim(std::ctime(&nowTime));
        
        if (!create_folder_flag_old && create_folder_flag)
        {
            index = 0;
            count_index = 0;
            create_folder_flag_old = create_folder_flag;
            
            if (enable_use_input_folder_name)
                folder_name = input_folder_name;
            else
                folder_name = folder_name_temp;
            folder_name = "/media/nvidia/ssd/data/" + folder_name;
            
            const char* folder_name_ = folder_name.c_str();
            if (mkdir(folder_name_, 0777) == -1) 
                cerr << "Error :  " << strerror(errno) << endl; 
            else
                cout << "Directory created\n";           
        }
        
        else if(create_folder_flag_old && !create_folder_flag)
            create_folder_flag_old = create_folder_flag;
        
        //         cap1.grab();
        //         cap2.grab();
        //         cap1.retrieve(frame);
        //         cap2.retrieve(frame2);
        if (!use_img)
        {
            cam.snapImage(60);
            cam2.snapImage(60);
            
            frame.create( cam.getHeight(),cam.getWidth(),CV_8UC(cam.getBytesPerPixel()));
            memcpy( frame.data, cam.getImageData(), cam.getImageDataSize());
            
            frame2.create( cam2.getHeight(),cam2.getWidth(),CV_8UC(4));
            memcpy( frame2.data, cam2.getImageData(), cam2.getImageDataSize());
            
             // save image to the local drive
            if (create_folder_flag || use_old_folder_flag)
            {
                
                count_index++;
                if (count_index%skippedImg == 0)
                {
                    index++;
                    
                    string imgLeft_name  = folder_name + "/left"  + to_string(index) + ".jpg"; 
                    string imgRight_name = folder_name + "/right" + to_string(index) + ".jpg"; 
                    
                    cout  << "imgLeft_name is ' " << imgLeft_name << " '" << endl;
                    imwrite(imgLeft_name,  frame);
                    imwrite(imgRight_name, frame2);
                }
                else
                    cout << "create folder is false" << endl;
            }
            
            //         std::string img_name = "left" + std::to_string(count) + ".jpg";
            //         std::string img_name2 = "right" + std::to_string(count) + ".jpg";
            //         count++;
            //         cout << "frame.size = " << frame.size() << endl;
//                       cv::Mat img(frame), img2(frame2);
//                     cv::resize(img, img, cv::Size(img.cols/2, img.rows/2));
//                     cv::resize(img2, img2, cv::Size(img2.cols/2, img2.rows/2));
//                     
//                     cv::imshow("cam", img);
//                     cv::imshow("cam2", img2);
//                     cv::waitKey(1);
//             
        }
        else
        {
            
            ifstream myReadFile;
            myReadFile.open("src/config/inputimage.txt");
            string output;
            if (myReadFile.is_open()) 
            {
                while (!myReadFile.eof()) 
                {
                    myReadFile >> output;
                    //                     cout<<output;
                }
            }
            myReadFile.close();
            output = output + "/";
            
            string folder_name_ = output; // this pin pin_insulator_exposure_200ms_small_orientation
            
            //         string folder_name_ = "/media/nvidia/ssd/data/exposure_850ms_4meter_largerAngle_7PM/";
            string folder_name_left = folder_name_ + "left" + to_string(count_img) + ".jpg";
            string folder_name_right = folder_name_ + "right" + to_string(count_img) + ".jpg";
            cout << "left image is " << folder_name_left << endl;
            count_img++;
            if (frame.empty())
                count_img = 1;
            
            frame = imread(folder_name_left);
            frame2 = imread(folder_name_right);
            
            
            
        }
        
        
        
        
        
        if(!frame.empty() & !frame2.empty())
        {
            ros::Time capture_time = ros::Time::now();
            msg.capture_time = ros::Time::now();
            // here is to rectify the image
            
            int calib_width = w, calib_height = h, out_width = (int)(w / res_div), out_height = (int)(h / res_div);
            calib_img_size = Size(calib_width, calib_height);
            out_img_size = Size(out_width * 1, out_height * 1);
            findRectificationMap(calib_file, out_img_size);
            
            
            cv::Mat src_color_left, src_color_right;
            //                 cout << "here 1" << endl;
            remap(frame, src_color_left, lmapx, lmapy, cv::INTER_LINEAR);
            remap(frame2, src_color_right, rmapx, rmapy, cv::INTER_LINEAR);
            
            // creating the vector
            Point my_array[16];
            Point point;
            vector<float> Q_vector;
            
//             Q_vector.push_back(Q.at<double>(0, 0));
//             Q_vector.push_back(Q.at<double>(0, 1));
//             Q_vector.push_back(Q.at<double>(0, 2));
//             Q_vector.push_back(Q.at<double>(0, 3));
//             Q_vector.push_back(Q.at<double>(1, 0));
//             Q_vector.push_back(Q.at<double>(1, 1));
//             Q_vector.push_back(Q.at<double>(1, 2));
//             Q_vector.push_back(Q.at<double>(1, 3));
//             Q_vector.push_back(Q.at<double>(2, 0));
//             Q_vector.push_back(Q.at<double>(2, 1));
//             Q_vector.push_back(Q.at<double>(2, 2));
//             Q_vector.push_back(Q.at<double>(2, 3));
//             Q_vector.push_back(Q.at<double>(3, 0));
//             Q_vector.push_back(Q.at<double>(3, 1));
//             Q_vector.push_back(Q.at<double>(3, 2));
//             Q_vector.push_back(Q.at<double>(3, 3));
            
             for (int i = 0; i < 4; i++)
            {
             
                for(int j = 0; j < 4; j++)
                {
                    Q_vector.push_back(Q.at<double>(i, j));
                    
                }
            }
            
//             cout << Q << endl;
            
            for (int i=0; i < 16; i++) {
                point.x = Q_vector[i];
                point.y = i;
                my_array[i] = point;
            }
            std::vector<Point> my_vector (my_array, my_array + sizeof(my_array) / sizeof(Point));
            
            
            int i = 0;
            for (std::vector<Point>::iterator it = my_vector.begin(); it != my_vector.end(); ++it) {
                geometry_msgs::Point point;
                point.x = (*it).x;
                
                point.y = (*it).y;
                point.z = 0;
                msg.points.push_back(point);
                msg.Q_array.push_back(Q_vector[i]);
                i++;
            }
            
//            
            
            
//             ROS_INFO("%d", msg.Q_array[9]);
            pub.publish(msg);
            ++count;
            
        
            raw_out_msg.header       = std_msgs::Header(); // New Header
            raw_out_msg.header.stamp = capture_time;
            //raw_out_msg.encoding     = sensor_msgs::image_encodings::MONO8; // for Raw data
            if (!use_img)
            {
                raw_out_msg.encoding     = sensor_msgs::image_encodings::BGRA8; // for Raw data
            }
            else
            {
                raw_out_msg.encoding     = sensor_msgs::image_encodings::BGR8; // for Raw data
            }
            raw_out_msg.image        = src_color_left; //output_im;//img;//cv_ptr->image;//output_img;
            pub_cam_msg.publish(raw_out_msg.toImageMsg());
            
            raw_out_msg_right.header       = std_msgs::Header(); // New Header
            raw_out_msg_right.header.stamp = capture_time;
            //raw_out_msg_right.encoding     = sensor_msgs::image_encodings::MONO8; // for RAW data
            if (!use_img)
            {
                raw_out_msg_right.encoding     = sensor_msgs::image_encodings::BGRA8; // for Raw data
            }
            else
            {
                raw_out_msg_right.encoding     = sensor_msgs::image_encodings::BGR8; // for Raw data
                
            }
            raw_out_msg_right.image        = src_color_right; //output_im;//img;//cv_ptr->image;//output_img;
            pub_cam_msg_right.publish(raw_out_msg_right.toImageMsg());
            
            ci_left=caminfo_left.getCameraInfo();
            ci_left.header.stamp = ros::Time::now();
            ci_left.header.frame_id = "stereo";
            pub_cam_info_left.publish(ci_left);        
            
            
           
            //             else if(use_old_folder_flag)
            //             {
            //                 
            //                 
            //                 
            //                 
            //             }
            
           
            
            waitKey(1);
        }
        else
        {
            cout << "EMPTY FRAME!" << endl;
        }        
        
        //         if (xiH || xiH2)
        //         {
        //             xiCloseDevice(xiH);
        //             xiCloseDevice(xiH2);
        //             printf("Done\n");
        //         }
        
        loop_rate.sleep();
        spinOnce();
    }
    return 0;
}
