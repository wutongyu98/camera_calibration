#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "tcamimage.h"


using namespace gsttcam;
using namespace std;
using namespace cv;
int exposure_time = 5000, w = 1440, h = 1080;


int main(int argc, char** argv)
{
    gst_init(&argc, &argv);

    // Initialize our TcamCamera object "cam" with the serial number
    // of the camera, which is to be used in this program.
    // my camera is here 03020388 and 03020395
    TcamImage cam("03020388"); 
    TcamImage cam2("03020395"); 
    
    cam.set_capture_format("BGRx", FrameSize{1440,1080}, FrameRate{30,1});
    cam2.set_capture_format("BGRx", FrameSize{1440,1080}, FrameRate{30,1});
    
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
    
    
    
    Mat frame, frame2;    
    
    while (1) 
    {
        
        cam.snapImage(60);
        cam2.snapImage(60);
        
        frame.create( cam.getHeight(),cam.getWidth(),CV_8UC(cam.getBytesPerPixel()));
        memcpy( frame.data, cam.getImageData(), cam.getImageDataSize());
        
        frame2.create( cam2.getHeight(),cam2.getWidth(),CV_8UC(4));
        memcpy( frame2.data, cam2.getImageData(), cam2.getImageDataSize());
        
        
        if(!frame.empty() & !frame2.empty())
        {
            imshow("left", frame);
            imshow("right", frame2);
            waitKey(1);
        }
        else
            cout << "EMPTY FRAME!" << endl;
        
    }
    return 0;
}
