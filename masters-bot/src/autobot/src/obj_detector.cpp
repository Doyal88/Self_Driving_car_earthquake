/*
 * http://github.com/dusty-nv/jetson-inference
 */

// #include "gstCamera.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <autobot/compound_img.h>

#include <autobot/detected_img.h>
#include <autobot/enable_lidar.h>
#include <autobot/drive_param.h>
#include <autobot/bounding_box.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <boost/make_shared.hpp>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <string>
#include <iostream>
#include <glob.h>
#include <jetson-inference/cudaMappedMemory.h>
#include <jetson-inference/cudaNormalize.h>
#include <jetson-inference/cudaFont.h>

#include <jetson-inference/detectNet.h>
#include <jetson-inference/segNet.h>

#define DEFAULT_CAMERA 0   //change 0 for zed camera images and 1 for static saved images in given folder 

using namespace std;
using namespace cv;

bool signal_recieved = false;

//void sig_handler(int signo)
//{
    //if( signo == SIGINT )
    //{
        //printf("received SIGINT\n");
        //signal_recieved = true;
    //}
//}

static const std::string OPENCV_WINDOW = "Image post bridge conversion";
static const std::string OPENCV_WINDOW2 = "Image post bit depth conversion";
static const std::string OPENCV_WINDOW3 = "Image post color conversion";
static const std::string OPENCV_WINDOW4 = "Image window";
//const int height = 320;//image.rows;
//const int width = 620;//image.cols;    

//void dfs(Vec4f color, int visited[height][width], int& count,int r,int c,int dr[4],int dc[4]);

class obj_new
{
    volatile int frame_num = 0;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber image_sub_;
    //detectNet* net = NULL;
    segNet* sNet = NULL;
    std::chrono::steady_clock::time_point prev;
    cv::Mat cv_im;
    cv_bridge::CvImagePtr cv_ptr;
    float4* gpu_data = NULL;

    float* outCPU    = NULL;
    float* outCUDA   = NULL;

    uint32_t imgWidth;
    uint32_t imgHeight;
    size_t imgSize;     
    bool displayToScreen = true;
    bool flag =  true;

    ros::Publisher detect_img_pub;
    ros::Publisher drive_pub_;
    ros::Publisher lidar_pub_;
    
    int color_road[3] = {128, 64, 128};
    glob_t glob_result;
    std::string base= "/home/ubuntu/zed-opencv/build/test_data/*";
    int counter = 0;
    int itt = 5;
    int itt_i = 0;
    autobot::drive_param driveParam;
    autobot::enable_lidar enableLidar;
    
    public:
    obj_new(int argc, char** argv, bool display) : it_(nh_)
    {
        cout << "start constructor" << endl;
        image_sub_ = nh_.subscribe("/compound_img", 1,&obj_new::imageCb, this);
        detect_img_pub = nh_.advertise<autobot::detected_img>("/detected_image", 2);
        drive_pub_ = nh_.advertise<autobot::drive_param>("/drive_parameters", 10);
        lidar_pub_ = nh_.advertise<autobot::enable_lidar>("/e_lidar", 10);
        displayToScreen = display;

        if (displayToScreen) {
        
            cout << "In if" << endl;
           // cv::namedWindow(OPENCV_WINDOW);
          //  cv::namedWindow( OPENCV_WINDOW, cv::WINDOW_AUTOSIZE );
          //  cout << "Named a window" << endl;
            prev = std::chrono::steady_clock::now();
         }

        sNet = segNet::Create(argc, argv);
        cout << "Created DetectNet" << endl;

        sNet->SetGlobalAlpha(120);
        
        if( !sNet )
        {
            printf("obj_detect:   failed to initialize imageNet\n");
        }
        // this part is being done in segNet->Create()
        
        if( !cudaAllocMapped((void**)&outCPU, (void**)&outCUDA, 961 * 541 * sizeof(float) * 4) )
	    {
		    printf("segnet:  failed to allocate CUDA memory for output image (%ix%i)\n", imgWidth, imgHeight);
	    }
	    
	   /* if( !cudaAllocMapped((void**)&outCPU, (void**)&outCUDA, 376 * 672 * sizeof(float) * 4) )
	    {
		    printf("segnet:  failed to allocate CUDA memory for output image (%ix%i)\n", imgWidth, imgHeight);
	    }*/
        memset(&glob_result, 0, sizeof(glob_result));
        int return_value = glob(base.c_str(),GLOB_TILDE,NULL,&glob_result);
        if(return_value != 0) {
            globfree(&glob_result);
            cout << "Fial to get files" << endl;
         }
    }
    
 

    void findCarAttribute_new2(Mat image){
        int height = image.rows;
        int width = image.cols;
        int speed = 2;
        int angle = 90;
        bool flag_stop = false;
        bool flag = true;
        bool flag_sharp = false;
        int count = 0;
        bool flag_mild = false;
        enableLidar.enable_lidar = 1;


        /* Very nearer area */
        int arr_count[3] = {0,0,0};
        int k =0;
        for(int i =(height-((int)(1*height/10))); i<height;i++){
            for(int j = (int)(2*width/10);j<(int)(8*width/10);j++){
               Vec4f color = image.at<Vec4f>(i,j);
                if(!((color[0] == color_road[0]) && (color[1] == color_road[1]) && (color[2] == color_road[2]))){
                     arr_count[k]++;
                }
                if(j==(int)(4*width/10)){
                 k=1;
                } else if(j==(int)(6*width/10)) {
                 k=2;
                } else if(j==(int)(8*width/10)-1){
                 k=0;
                }
            }   
        }  
        cout << "nearer : " << arr_count[0] << " " << arr_count[1] << " " << arr_count[2] << endl;
        if(arr_count[1] <= 500){
            if((arr_count[0] <= 500) && (arr_count[2] <= 500)) {
                /* all empty*/
                speed = 2;
                angle = 90;
                flag_stop = true;
            } else if((arr_count[0] <= 500) && (arr_count[2] > 500)) {
                // left
                speed = 1;
                angle = 30; 
            } else if((arr_count[0] > 500) && (arr_count[2] <= 500)){
                // right
                speed = 1;
                angle = 150;            
            } else { 
                //compare count and turn
                /*if(arr_count[0] > arr_count[2]) {
                   angle = 150; 
                } else {
                    angle = 30;
                }*/
                angle = 90;
                speed = 0;              
            }
        } else {
            if((arr_count[0] <= 500) && (arr_count[2] <= 500)) {
                /* left priority turn*/
                //speed = 1;
                //angle = 30;
                if(arr_count[0] > arr_count[2]) {
                   angle = 150; 
                } else {
                    angle = 30;
                }
                speed = 1;            
            } else if((arr_count[0] <= 500) && (arr_count[2] > 500)) {
                // left
                speed = 1;
                angle = 30; 
            } else if((arr_count[0] > 500) && (arr_count[2] <= 500)){
                // right
                speed = 1;
                angle = 150;            
            } else { 
                //all block
                speed = 0;
                angle = 90;            
            }
        }
        
        arr_count[0] = 0,arr_count[1]=0,arr_count[2] = 0;
        k =0;         
        /* Middle Area */
        if(flag_stop == true)
        {
            for(int i =(height-((int)(3*height/10))); i<(height-((int)(1*height/10)));i++)
            {
                for(int j = (int)(1*width/10);j<(int)(9*width/10);j++)
                {
                    Vec4f color = image.at<Vec4f>(i,j);
                    if(!((color[0] == color_road[0]) && (color[1] == color_road[1]) && (color[2] == color_road[2])))
                    {
                        arr_count[k]++;
                    }
                    if(j==(int)(3*width/10)){
                        k=1;
                    } else if(j==(int)(7*width/10)) {
                        k=2;
                    } else if(j==(int)(9*width/10)-1){
                        k=0;
                    }
                }   
            }       
       
        cout << "middle : " << arr_count[0] << " " << arr_count[1] << " " << arr_count[2] << endl;

      if(arr_count[1] <= 2000){
           if((arr_count[0] <= 800) && (arr_count[2] <= 800)) {
                /* all empty*/
                speed = 2;
                angle = 90;
                flag_sharp = true;
           } else if((arr_count[0] <= 800) && (arr_count[2] > 800)) {
                // left
                speed = 1;
                angle = 45; 
            } else if((arr_count[0] > 800) && (arr_count[2] <= 800)){
                // right
                speed = 1;
                angle = 135;            
            } else { 
                //compare count and turn
                if(arr_count[0] > arr_count[2]) {
                   angle = 135; 
                } else {
                    angle = 45;
                }
                speed = 1;            
                //angle = 90;
                //speed = 1;               
            }
        } else {
            if((arr_count[0] <= 800) && (arr_count[2] <= 800)) {
                /* left priority turn*/
                //speed = 1;
                //angle = 45;
                if(arr_count[0] > arr_count[2]) {
                   angle = 135; 
                } else {
                    angle = 45;
                }
                speed = 1;            
            } else if((arr_count[0] <= 800) && (arr_count[2] > 800)) {
                // left
                speed = 1;
                angle = 45; 
            } else if((arr_count[0] > 800) && (arr_count[2] <= 800)){
                // right
                speed = 1;
                angle = 135;            
            } else { 
               //compare count and turn
                if(arr_count[0] > arr_count[2]) {
                   angle = 135; 
                } else {
                    angle = 45;
                }
                speed = 1;            
            }
        }


        arr_count[0] = 0,arr_count[1]=0,arr_count[2] = 0;
        k =0;         
        /* far Area */
        if(flag_stop == true && flag_sharp == true)
        {
            for(int i = (height-((int)(5*height/10))); i < (height-((int)(3*height/10))); i++)
            {
                for(int j = 0; j<width; j++)
                {
                    Vec4f color = image.at<Vec4f>(i,j);
                    
                    if(!((color[0] == color_road[0]) && (color[1] == color_road[1]) && (color[2] == color_road[2])))
                    {
                        arr_count[k]++;
                    }
                    
                    if(j==(int)(3*width/10))
                    {
                        k=1;
                    } 
                    else if(j==(int)(7*width/10))
                    {
                        k=2;
                    } 
                    else if(j==(int)(width-1))
                    {
                        k=0;
                    }
                }   
            }       
        
        cout << "far : " << arr_count[0] << " " << arr_count[1] << " " << arr_count[2] << endl;
                
        if(arr_count[1] <= 800){
            //if((arr_count[0] <= 600) && (arr_count[2] <= 600)) {
                /* all empty*/
                speed = 2;
                angle = 90;
                flag_mild = true;
           /* } else if((arr_count[0] <= 600) && (arr_count[2] > 600)) {
                // left
                speed = 1;
                angle = 60; 
            } else if((arr_count[0] > 600) && (arr_count[2] <= 600)){
                // right
                speed = 1;
                angle = 120;            
            } else { 
                //compare count an turn
                angle = 90;
                speed = 1;   
            }*/
        } else {
            if((arr_count[0] <= 600) && (arr_count[2] <= 600)) {
                /* left priority turn*/
                //speed = 1;
                //angle = 60;
                if(arr_count[0] > arr_count[2]) {
                   angle = 120; 
                } else {
                    angle = 60;
                }
                speed = 1;            
            } else if((arr_count[0] <= 600) && (arr_count[2] > 600)) {
                // left
                speed = 1;
                angle = 60; 
            } else if((arr_count[0] > 600) && (arr_count[2] <= 600)){
                // right
                speed = 1;
                angle = 120;            
            } else { 
                //compare count and turn
                if(arr_count[0] > arr_count[2]) {
                   angle = 120; 
                } else {
                    angle = 60;
                }
                speed = 1;            
            }
        }
    }        
}
        if(flag_sharp && flag_stop && flag_mild){
            enableLidar.enable_lidar = 0;
            cout << "fs: " << flag_sharp << "fm: " << flag_mild << endl; 
        }
        
        switch (speed) {
            case 0 :
                driveParam.velocity = 9830;
                break;
            case 1:
                driveParam.velocity = 10070;
                break;
            case 2:
                driveParam.velocity = 10070;
                break;
            default : 
                driveParam.velocity = 9830;
                break;
        }    
        driveParam.angle = angle - 90 ;
        cout <<"Final Angle " << angle <<" and speed : " << speed << endl;
        
        lidar_pub_.publish<autobot::enable_lidar>(enableLidar);
        
        if(enableLidar.enable_lidar == 1)
            drive_pub_.publish<autobot::drive_param>(driveParam); 
    }


    ~obj_new()
    {
        CUDA(cudaFree(gpu_data));
        delete gpu_data;
        globfree(&glob_result);
        //if (displayToScreen) {
        //   cv::destroyWindow(OPENCV_WINDOW);
        //}
    }
    
    void imageCb(const autobot::compound_img& cp_msg)
    {
        //cout << "image detected............." << endl;
        if(DEFAULT_CAMERA > 0) {
            cout << "img_name: " << glob_result.gl_pathv[counter] << endl;
            cv_im = imread(glob_result.gl_pathv[counter],CV_LOAD_IMAGE_COLOR);
            cv::resize(cv_im,cv_im,cv::Size(961,541));
            itt_i++;
            
            if(itt_i == itt){
                itt_i = 0;
                counter++;
            }
            
            if(counter == glob_result.gl_pathc){
                counter = 0;
            }
            
        } 
        else {
            //cout << "Camera Input";
            try
            {
                cv_ptr = cv_bridge::toCvCopy(cp_msg.img, sensor_msgs::image_encodings::BGR8);
                cv_im = cv_ptr->image;
                cv::resize(cv_im,cv_im,cv::Size(961,541));
            }
            catch (cv_bridge::Exception& e)
            {
                cout << "cv_bridge exception";
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }
        cv_im.convertTo(cv_im,CV_32FC3);
        // convert color
        cv::cvtColor(cv_im,cv_im,CV_BGR2RGBA);
       
        if (!gpu_data) {
            ROS_INFO("first allocation");
            CUDA(cudaMalloc(&gpu_data, cv_im.rows*cv_im.cols * sizeof(float4)));
        } else if (imgHeight != cv_im.rows || imgWidth != cv_im.cols) {
            ROS_INFO("re allocation");
            // reallocate for a new image size if necessary
            CUDA(cudaFree(gpu_data));
            CUDA(cudaMalloc(&gpu_data, cv_im.rows*cv_im.cols * sizeof(float4)));
        }
        
        imgHeight = cv_im.rows;
        imgWidth = cv_im.cols;
        imgSize = cv_im.rows*cv_im.cols * sizeof(float4);
        float4* cpu_data = (float4*)(cv_im.data);
        //cv::Mat dest;
        //cout << "Height : " << imgHeight <<  " and width " << imgWidth << endl;
        
        // copy to device
        CUDA(cudaMemcpy(gpu_data, cpu_data, imgSize, cudaMemcpyHostToDevice));

        // process image overlay
		     if( !sNet->Overlay((float*)gpu_data, outCUDA , imgWidth, imgHeight) )
		     {
		        cout << "segnet :  failed to process segmentation overlay" << endl;
		     }
		     else
		     {
		        cv::Mat dest(imgHeight, imgWidth, CV_32FC4, (float*)outCUDA );
		        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
                float fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(now - prev).count() ;
                prev = now;
		        
		        //cout << "Execution frame per second : " << fps << endl ;
		        //cout <<"Destination file : " << dest.rows << "  " << dest.cols << endl;
                
                findCarAttribute_new2(dest);
                
		    }
		    
            // cv::imshow(OPENCV_WINDOW, dest); 
            
    }

};

  


int main( int argc, char** argv ) {
    cout << "starting node" << endl;
    printf("obj_detect\n  args (%i):  ", argc);

    ros::init(argc, argv, "obj_detector");
    ros::NodeHandle nh;

    bool displayToScreen = false;

    for( int i=0; i < argc; i++ ) {
        printf("%i [%s]  ", i, argv[i]);
        if (strcmp(argv[i], "--DISPLAY") == 0) { 
           // displayToScreen = true;
        }
    }

    printf("\n\n");

   // ObjectDetector ic(argc, argv, displayToScreen);
    obj_new ic(argc, argv, displayToScreen);
    ros::Duration(0.5).sleep();
    ros::spin();

    return 0;
}

