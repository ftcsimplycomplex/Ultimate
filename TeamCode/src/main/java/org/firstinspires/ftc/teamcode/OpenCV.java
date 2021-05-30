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


    private final int FRAME_WIDTH = 640;
    private final int FRAME_HEIGHT = 480;
    private final int HORIZON_HEIGHT = 200;

    private OpenCvWebcam camera;

    private int visionState;

    public void initVision(){
        visionState = 1;       // Initialize the pipeline state machine.  Redundant: we do it in the constructor.
    }

    public void runVision(){
        visionState = 3;
    }

    /*
     * "Getter" method to return target zone A, B, or C as a string.
     *
     * Hue, from HSV, for a gray field is substantially larger numerically than hue from an orange field.
     *
     * If the Start Stack hue (from HSV) post-randomization is below, or not more than 1.5 times greater
     * than the pre-randomization hue, we consider the start stack unchanged (four rings, Target Zone C).
     *
     * Otherwise, we look only at the bottommost 1/4 of the rectangle, and make the same range comparison.
     * This way, we don't have the area above a remaining ring overwhelming the hue averaging process.
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

        private Mat blurInput       = new Mat(); // Working copy; keeps submats out of input buffer
        private Mat convertedInput  = new Mat(); // Working copy; keeps submats out of input buffer
        private Mat HSVInput        = new Mat(); // Working copy; keeps submats out of input buffer
        private Mat mask            = new Mat();
        private Mat canny           = new Mat();
        private Mat dilated         = new Mat();
        private Mat hierarchy       = new Mat();

        private List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        private Mat hueSample = new Mat();
        private Mat hueChannel = new Mat();

        private Mat hueSampleBottomQuarter = new Mat();
        private Mat hueChannelBottomQuarter = new Mat();

        Rect startStack;
        Rect bottomQuarter;

        boolean initStackFound = false;


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
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * processFrame() is called per-image-frame, from the webcam.  What happens to the frame is governed
             * by a state machine. State 1 is set by the Constructor, or by a call to initVision().  State 2 is
             * set by the code executed in State 1, once the Start Stack rectangle location is established.
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
                     * TL;DR: find the Start Stack rectangle by color, location, and geometry.
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

                        startStack = Imgproc.boundingRect(contour);

                        if(startStack.width > startStack.height && startStack.y > HORIZON_HEIGHT && startStack.area() > 5000){
                            initStackFound = true;
                            visionState = 2;
                            initStackAvg = avgHue();

                            bottomQuarter = startStack.clone();
                            bottomQuarter.height = startStack.height/4;
                            bottomQuarter.y = startStack.y + (startStack.height*3)/4;

                            initBottomQuarterAvg = avgHueBottomQuarter();
                            break;
                        }
                    }
                    break;
                case 2:
                    /*
                     * State 2: The Start Stack location is known.  Loop around testing for changes.
                     * Draw the horizon line for the camera preview unconditionally; draw the Start
                     * Stack rectangle if one has been found (that should always be the case if this
                     * code is running).  Test for changes: until randomization it will always see
                     * no changes (target zone C).  This will change if and when rings are removed for
                     * target zones A or B.
                     *
                     * Pipeline stays in state 2 until OpMode stops it with a call to stopVision().
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
            }

            return input;
        }
    }
}
