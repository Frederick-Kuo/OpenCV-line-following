// Include files for required libraries
#include <stdio.h>

#include <iostream>
#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"

//using namespace std;


//look into erode/dilate functions

void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV
    //Pi2c car(0x07); // Configure the I2C interface to the Car as a global variable

}

int main( int argc, char** argv )
{
    setup();    // Call a setup function to prepare IO and devices

    cv::namedWindow("Photo");   // Create a GUI window called photo
    float error;
    float errorLast;
    float errorSum;
    int u;
    int Kp=15;
    int Ki=0;
    int Kd=0; //PID values


    Pi2c arduino(7); //sets up i2c, address 7 (also in arduino code)
    while(1)    // Main loop to perform image processing
    {
        Mat frame;

        while(frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable

        //cv::imshow("Photo", frame); //Display the image in the window

        cv:: Mat frame2;

        cv::flip(frame,frame2,-1);

        Mat green;
        cvtColor(frame2,green,COLOR_BGR2HSV);
        //inRange(green,Scalar(40,50,50), Scalar(80,255,255),green); //GREEN
	//inRange(color,Scalar(0,50,50), Scalar(20,255,255),detect); //RED
	//inRange(color,Scalar(100,50,50), Scalar(140,255,255),detect); //BLUE
	//inRange(color,Scalar(25,50,50), Scalar(35,255,255),detect); //YELLOW
        inRange(green,Scalar(0,0,0), Scalar(179,255,51),green); //BLACK


        imshow("GreenLine",green);

        //Mat L5p;
        //L5p=frame2(Rect(0,0,32,24));

        Mat L5p = green(Range(0,240),Range(0,32));
        imshow("L5",L5p);
        Mat L4p = green(Range(0,240),Range(32,64));
        imshow("L4",L4p);
        Mat L3p = green(Range(0,240),Range(64,96));
        imshow("L3",L3p);
        Mat L2p = green(Range(0,240),Range(96,128));
        imshow("L2",L2p);
        Mat L1p = green(Range(0,240),Range(128,160));
        imshow("L1",L1p);
        Mat R1p = green(Range(0,240),Range(160,192));
        imshow("R1",R1p);
        Mat R2p = green(Range(0,240),Range(192,224));
        imshow("R2",R2p);
        Mat R3p = green(Range(0,240),Range(224,256));
        imshow("R3",R3p);
        Mat R4p = green(Range(0,240),Range(256,288));
        imshow("R4",R4p);
        Mat R5p = green(Range(0,240),Range(288,320));
        imshow("R5",R5p);

        int l5w= -4.5;
        int l4w= -3.5;
        int l3w= -2.5;
        int l2w= -1.5;
        int l1w= -0.5;

        int r1w= 0.5;
        int r2w= 1.5;
        int r3w= 2.5;
        int r4w= 3.5;
        int r5w= 4.5;

        int l5n = countNonZero(L5p); //counts the NUMBER of white pixels
        int l4n = countNonZero(L4p);
        int l3n = countNonZero(L3p);
        int l2n = countNonZero(L2p);
        int l1n = countNonZero(L1p);

        int r1n = countNonZero(R1p);
        int r2n = countNonZero(R2p);
        int r3n = countNonZero(R3p);
        int r4n = countNonZero(R4p);
        int r5n = countNonZero(R5p);

        //if ((l5n && l4n && l3n && l2n &&l1n &&r1n && r2n && r3n &&r4n &&r5n) <100000)
        if ((l5n + l4n + l3n + l2n + l1n + r1n + r2n + r3n + r4n + r5n) <100)
        {
            u =99;
        }

        else
        {
            errorLast=error;
            error=((l5n*l5w+l4n*l4w+l3n*l3w+l2n*l2w+l1n*l1w+r5n*r5w+r4n*r4w+r3n*r3w+r2n*r2w+r1n*r1w)/(l5n+l4n+l3n+l2n+l1n+r5n+r4n+r3n+r2n+r1n+1));
            errorSum=errorSum+error;
            u=(Kp*error)+(Ki*errorSum)+(Kd*(error-errorLast));

        }

        arduino.i2cWriteArduinoInt(u);


        //printf(l5n);
        //cout<<l5n;
        //cout<<"test";
        //printf(l5n, %d);
        //printf("test");

        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)

        key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
        if (key == 27)
            break;
	}

	closeCV();  // Disable the camera and close any windows

	return 0;
}



