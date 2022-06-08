#include <stdio.h>

#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"

Pi2c car(0x22); // Configure the I2C interface to the Car as a global variable
int lowH = 0, highH = 178, lowS = 0, highS = 254, lowV = 0, highV = 121;    // Initialise some variables for dark HSV limits



void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV
}

int main( int argc, char** argv )
{
    setup();    // Call a setup function to prepare IO and devices

    // Create arduino object, assign address
    Pi2c arduino(7);


    cv::namedWindow("Photo");   // Create a GUI window called photo
    cv::namedWindow("HSVTool"); // Create a GUI window called HSVTool
    cv::namedWindow("P1"); // Create a GUI window called HSVTool
    cv::namedWindow("P2"); // Create a GUI window called HSVTool
    cv::namedWindow("P3"); // Create a GUI window called HSVTool
    cv::namedWindow("P4"); // Create a GUI window called HSVTool
    cv::namedWindow("P5"); // Create a GUI window called HSVTool
    cv::namedWindow("P6"); // Create a GUI window called HSVTool
    cv::namedWindow("P7"); // Create a GUI window called HSVTool


    while(1)    // Main loop to perform image processing
    {

        Mat frame;

        while(frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable

        Mat frameflip;
        Mat frameflipflip;
        flip(frame,frameflip,0);
        flip(frameflip,frameflipflip,1);

        cv::imshow("Photo", frameflipflip); //Display the image in the window

        Mat frameHSV;       // Convert the frameflipflip to HSV and apply the limits
        cvtColor(frameflipflip, frameHSV, COLOR_BGR2HSV);
        inRange(frameHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), frameHSV);
        imshow("HSVTool", frameHSV); //Display the image in the window

        Mat partA;
        partA = frameHSV(Rect(0, 210, 320, 30));
        cv::imshow("P1", partA); //Display the image in the window


        Mat partB;
        partB = partA(Rect(0, 0, 60, 30));
        cv::imshow("P2", partB); //Display the image in the window
        int numNonZeroB = 0; // Create an integer to store the result
        numNonZeroB = countNonZero(partB); // Count the number of non-zero pixels
        arduino.i2cWriteArduinoInt(numNonZeroB); // Send the data via I2c


        Mat partC;
        partC = partA(Rect(60, 0, 50, 30));
        cv::imshow("P3", partC); //Display the image in the window
        int numNonZeroC = 0; // Create an integer to store the result
        numNonZeroC = countNonZero(partC); // Count the number of non-zero pixels

        Mat partD;
        partD = partA(Rect(110, 0, 50, 30));
        cv::imshow("P4", partD); //Display the image in the window
        int numNonZeroD = 0; // Create an integer to store the result
        numNonZeroD = countNonZero(partD); // Count the number of non-zero pixels

        Mat partE;
        partE = partA(Rect(160, 0, 50, 30));
        cv::imshow("P5", partE); //Display the image in the window
        int numNonZeroE = countNonZero(partE);

        Mat partF;
        partF = partA(Rect(210, 0, 50, 30));
        cv::imshow("P6", partF); //Display the image in the window
        int numNonZeroF = countNonZero(partF);

        Mat partG;
        partG = partA(Rect(260, 0, 60, 30));
        cv::imshow("P7", partG); //Display the image in the window
        int numNonZeroG = countNonZero(partG);
        arduino.i2cWriteArduinoInt(numNonZeroG);
        std::cout << numNonZeroB << "\t" << numNonZeroC << "\t" << numNonZeroD << "\t" << numNonZeroE << "\t" << numNonZeroF << "\t" << numNonZeroG << "\n";

        // Calculating PID

        // Weighted average
        float WAF = ((numNonZeroB * 160 + numNonZeroC * 106.7 + numNonZeroD * 53.3 + numNonZeroE *-53.3 + numNonZeroF * - 106.7 + numNonZeroG * - 160)/(numNonZeroB+numNonZeroC+numNonZeroD+numNonZeroE+numNonZeroF+numNonZeroG));
        std::cout<< "Weighted average" << "\t" <<WAF<<"\n";

        // Error Level
        float error = 160 + WAF;
        std::cout << "Error" << "\t" << error <<"\n";


        // PID

        int Kp = 25;
        int Ki = 0.1;
        int Kd = 120;
        float olderror = error;

        float P = error;
        float I = I + error;
        float D = error - olderror;
        float PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
        std::cout<< "PID" << "\t" << PIDvalue;

        //namedWindow("HSVTool", WINDOW_AUTOSIZE );
        //Mat comparison;     // Join the two into a single image
        //cvtColor(frameHSV, frameHSV, COLOR_GRAY2BGR);   // In range returns the equivalent of a grayscale image so we need to convert this before concatenation
        //hconcat(frameflip, frameHSV, comparison);



        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)

        key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
        if (key == 27)
            break;
	}

	closeCV();  // Disable the camera and close any windows

	return 0;
}
