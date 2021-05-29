package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.core.Core.inRange;

public class OpenCV {
    private static String ringPosition;

    // Exposed outside of the pipeline for telemetry
    private double topAverage;
    private double bottomAverage;

    double initStackAvg; //Was 15.6 last time tested
    double finalStackAvg;
    double initBottomQuarterAvg;
    double finalBottomQuarterAvg;

    /*
     * Telemetry from on-field tests show that a rectangle sample on an orange ring produces a topAverage / bottomAverage
     * of about 90, where no ring (no orange color) yields something around 125.  So, let's use 110 as the threshold,
     * above 100 is no ring, below 100 is ring.
     */
    private final double ORANGE_THRESHOLD = 110;
    private final int FRAME_WIDTH = 640;
    private final int FRAME_HEIGHT = 480;
    private final int HORIZON_HEIGHT = 200;

    private OpenCvWebcam camera;

    private int visionState;

    public void initVision(){
        visionState = 1;
    }

    public void runVision(){
        visionState = 3;
    }

    /*
    * "Getter" method to return target zone A, B, or C as a string
    */
    public String getPosition()
    {
        if(initStackAvg*1.5 > finalStackAvg){
            ringPosition = "C";
        }else{
            if(initBottomQuarterAvg*1.5 > finalBottomQuarterAvg){
                ringPosition = "B";
            }else{
                ringPosition = "A";
            }
        }
        return ringPosition;
    }

    public double getTopAverage() {return topAverage;}  // "Getter" method for telemetry
    public double getBottomAverage() {return bottomAverage;}  // "Getter" method for telemetry

    public double getInitStackAvg(){return initStackAvg;}
    public double getFinalStackAvg(){return finalStackAvg;}

    public double getInitStackAvgBottomQuarter(){return initBottomQuarterAvg;}
    public double getFinalStackAvgBottomQuarter(){return finalBottomQuarterAvg;}

 /*
 * Constructor: Gets and opens the webcam, sets the processing pipeline, and starts camera streaming.
 */
    public OpenCV(HardwareMap hardwareMap) {

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        camera.openCameraDevice();
        camera.setPipeline(new UltimatePipeline());
        camera.startStreaming(FRAME_WIDTH, FRAME_HEIGHT, OpenCvCameraRotation.UPRIGHT);

        visionState = 1;    // State machine variable for the pipeline
    }

    /**
     * stopDetect method.  Shuts it all down to save processing time after the robot starts moving.
     */
    void stopDetect(){
        camera.stopStreaming();
        camera.closeCameraDevice();
        //TODO Should we free up memory, too?
    }

    public class UltimatePipeline extends OpenCvPipeline {
        private Mat topRectangle    = new Mat();
        private Mat bottomRectangle = new Mat();

        private Mat blurInput       = new Mat(); // Working copy; keeps submats out of input buffer
        private Mat convertedInput  = new Mat(); // Working copy; keeps submats out of input buffer
        private Mat HSVInput        = new Mat(); // Working copy; keeps submats out of input buffer
        private Mat mask            = new Mat();
        private Mat canny           = new Mat();
        private Mat dilated         = new Mat();
        private Mat hierarchy       = new Mat();

        private List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        private Mat topSample = new Mat();      // For submat crops
        private Mat bottomSample = new Mat();   // For submat crops
        private Mat hueSample = new Mat();
        private Mat hueChannel = new Mat();

        private Mat hueSampleBottomQuarter = new Mat();
        private Mat hueChannelBottomQuarter = new Mat();

        double area;
        double peri;

        MatOfPoint2f approx;
        MatOfPoint2f newContour;
        int x,y,w,h;
        Rect startStack;
        Rect bottomQuarter;

        boolean initStackFound = false;

        private Rect bottomRect = new Rect(     // Dimensions and locations for sampling
                400,
                300,
                50,
                10
        );
        private Rect topRect = new Rect(        // Dimensions and locations for sampling
                400,
                240,
                50,
                10
        );

        private double avgHue(){
            hueSample = HSVInput.submat(startStack);

            Core.extractChannel(hueSample, hueChannel, 0);

            return Core.mean(hueChannel).val[0];
        }

        private double avgHueBottomQuarter(){
            hueSampleBottomQuarter = HSVInput.submat(bottomQuarter);

            Core.extractChannel(hueSampleBottomQuarter, hueChannelBottomQuarter, 0);

            return Core.mean(hueChannelBottomQuarter).val[0];
        }


        @Override
        public void init(Mat firstFrame)
        {
            Imgproc.GaussianBlur(firstFrame, blurInput, new Size(9,9), 0);
            Imgproc.cvtColor(blurInput, convertedInput, Imgproc.COLOR_RGB2YCrCb);    // Convert color space to working copy
            Imgproc.cvtColor(blurInput, HSVInput, Imgproc.COLOR_RGB2HSV);    // Convert color space to working copy


//            topSample = convertedInput.submat(topRect);         // Submat pointers are persistent
//            bottomSample = convertedInput.submat(bottomRect);   // Submat pointers are persistent
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * processFrame() is called per-image-frame, from the webcam.  What happens to the frame is governed
             * by a state machine, to match the stage of the game (i.e. "init" - look for the full four-ring start
             * stack.  "play" - randomization has occurred; the game has started.  Look at the start stack location
             * and act appropriately based on how, or if, it has changed since initialization.
             *
             */
            if (input.empty()) return input;

            switch (visionState){
                case 1:
                    /*
                     * State 1: entered upon "init" from Driver's Station.  (visionState variable set in Constructor.)
                     * Analyze the whole frame, set a mask based on Cr, Cb range, do edge detection, contours, and select
                     * the start stack based on location (below horizon line) and proportions of rectangular area.
                     *
                     */
                    Imgproc.GaussianBlur(input, blurInput, new Size(9,9), 0); //Blurs image to reduce noise

                    Imgproc.cvtColor(blurInput, convertedInput, Imgproc.COLOR_RGB2YCrCb);    // Convert color space to working copy
                    Imgproc.cvtColor(blurInput, HSVInput, Imgproc.COLOR_RGB2HSV);    // Convert color space to working copy

                    Scalar lower = new Scalar(0,   140,  0); //Lower color range
                    Scalar upper = new Scalar(255, 255,  115); //Higher color range



                    inRange(convertedInput, lower, upper, mask); //Creates mask using upper and lower

                    Imgproc.Canny(mask, canny, 50, 50); //Creates canny

                    Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * 2 + 1, 2 * 2 + 1),
                            new Point(2, 2));
                    Imgproc.dilate(canny, dilated, element);

                    Imgproc.findContours(canny, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE); //Creates list of contours


                    for(MatOfPoint contour: contours){
//                        area = Imgproc.contourArea(contour);
//                        if(area > 5000){
                        /*newContour = new MatOfPoint2f(contour);
                        peri = Imgproc.arcLength(newContour, true);
                        Imgproc.approxPolyDP(newContour, approx, peri, true);*/

                        startStack = Imgproc.boundingRect(contour);

                        if(startStack.width > startStack.height && startStack.y > HORIZON_HEIGHT && startStack.area() > 5000){
                            initStackFound = true;
                            visionState = 2;
                            initStackAvg = avgHue();

                            bottomQuarter = startStack.clone();
/*
                            bottomQuarter.x = startStack.x;
                            bottomQuarter.width = startStack.width;*/
                            bottomQuarter.height = startStack.height/4;
                            bottomQuarter.y = startStack.y + (startStack.height*3)/4;


                            initBottomQuarterAvg = avgHueBottomQuarter();
                            break;
                        }
//                        }
                    }
                    break;
                case 2:
                    /*
                     * State 2: Waiting for Start ("play" button on the Driver's Station.) Just return the camera
                     * image, with the horizon line drawn, and the start stack rectangle to confirm that it's been
                     * found.  This will show up during camera preview.  Transition to State 3 will be triggered
                     * by a call to runVision().  Note that there is no need to set a next state in this code block.
                     */
                    Imgproc.GaussianBlur(input, blurInput, new Size(9,9), 0); //Blurs image to reduce noise

                    Imgproc.cvtColor(blurInput, convertedInput, Imgproc.COLOR_RGB2YCrCb);    // Convert color space to working copy
                    Imgproc.cvtColor(blurInput, HSVInput, Imgproc.COLOR_RGB2HSV);    // Convert color space to working copy

                    Imgproc.line(input, new Point(0,HORIZON_HEIGHT), new Point(input.width()-1, HORIZON_HEIGHT),new Scalar(0,0,255), 4);

                    if(initStackFound){
                        Imgproc.rectangle(input, startStack, new Scalar(0,255,255),4);
                        finalStackAvg = avgHue();
                        finalBottomQuarterAvg = avgHueBottomQuarter();
                    }
                    break;
                case 3:
                    /*
                     * State 3: Entered after OpMode calls runVision(), immediately after Drive Team Coach presses "play".
                     * Analyze the average Hue inside the pre-established start stack rectangle, and compare it to the
                     * initial value.  Very close to initial value means it's still four rings.  Thresholds for one or zero
                     * rings to be determined empirically.
                     *
                     */
                    finalStackAvg = avgHue();
                    visionState = 4;
                    break;
                case 4:
                    /*
                     * State 4: Do nothing, just return the frame.  "Our work here is done."  OpMode should call stopDetect()
                     * to shut down the pipeline and free up resources.
                     *
                     */

                    break;
            }
            /*topSample = convertedInput.submat(topRect);         // Shouldn't be necessary according to the docs
            bottomSample = convertedInput.submat(bottomRect);   // Shouldn't be necessary according to the docs

            Imgproc.rectangle(input, topRect, new Scalar(0, 255, 0), 2);        // Draw rectangles on input buffer
            Imgproc.rectangle(input, bottomRect, new Scalar(0, 255, 0), 2);     // for drive team feedback

            Core.extractChannel(bottomSample, bottomRectangle, 2);
            Core.extractChannel(topSample, topRectangle, 2);

            bottomAverage = Core.mean(bottomRectangle).val[0];
            topAverage = Core.mean(topRectangle).val[0];

            if (topAverage < ORANGE_THRESHOLD && bottomAverage < ORANGE_THRESHOLD) {            // Both rectangles detect orange = 4 rings
                ringPosition = "C";
            } else if (topAverage > ORANGE_THRESHOLD && bottomAverage < ORANGE_THRESHOLD) {     // Top rectangle is not orange, bottom
                ringPosition = "B";                                                             // is orange = 1 ring
            } else {                    // default is no rings
                ringPosition = "A";
            }*/

            return input;
        }
    }
}
