package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class OpenCV {
    private static String ringPosition;

    // FBO telemetry
    private double topAverage;
    private double bottomAverage;

    private OpenCvWebcam camera;

    /*
    * "Getter" method to return target zone A, B, or C as a string
    */
    public String getPosition()
    {
        return ringPosition;
    }

    public double getTopAverage() {return topAverage;}  // For telemetry
    public double getBottomAverage() {return bottomAverage;}  // For telemetry

 /*
 * Constructor: Gets and opens the webcam, sets the processing pipeline, and starts camera streaming.
 */
    public OpenCV(HardwareMap hardwareMap) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        camera.openCameraDevice();
        camera.setPipeline(new UltimatePipeline());
        camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
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

        private Mat topRectangle = new Mat();
        private Mat bottomRectangle = new Mat();

        private Mat convertedInput = new Mat(); // Working copy; keeps submats out of input buffer

        private Mat topSample = new Mat();      // For submat crops
        private Mat bottomSample = new Mat();   // For submat crops

        private Rect bottomRect = new Rect(     // Dimensions and locations for sampling
                360,
                450,
                40,
                10
        );
        private Rect topRect = new Rect(        // Dimensions and locations for sampling
                360,
                400,
                40,
                10
        );

/* While we do something else for telemetry
        private double bottomAverage;
        private double topAverage;
*/

        @Override
        public void init(Mat firstFrame)
        {
            Imgproc.cvtColor(firstFrame, convertedInput, Imgproc.COLOR_RGB2YCrCb);    // Convert color space to working copy

            topSample = convertedInput.submat(topRect);         // Submat pointers are persistent
            bottomSample = convertedInput.submat(bottomRect);   // Submat pointers are persistent
        }

        @Override
        public Mat processFrame(Mat input) {
            if (input.empty()) return input;

            Imgproc.cvtColor(input, convertedInput, Imgproc.COLOR_RGB2YCrCb);    // Convert color space to working copy

            topSample = convertedInput.submat(topRect);         // Shouldn't be necessary according to the docs
            bottomSample = convertedInput.submat(bottomRect);   // Shouldn't be necessary according to the docs

            Imgproc.rectangle(input, topRect, new Scalar(0, 255, 0), 2);        // Draw rectangles on input buffer
            Imgproc.rectangle(input, bottomRect, new Scalar(0, 255, 0), 2);     // for drive team feedback

/*
            convertedInput.submat(topRect).copyTo(topSample);
            convertedInput.submat(bottomRect).copyTo(bottomSample);
*/

/*
            Core.extractChannel(input.submat(bottomRect), bottomRectangle, 2);
            Core.extractChannel(input.submat(topRect), topRectangle, 2);
*/
            // Try it this way:
            Core.extractChannel(bottomSample, bottomRectangle, 2);
            Core.extractChannel(topSample, topRectangle, 2);


            bottomAverage = Core.mean(bottomRectangle).val[0];
            topAverage = Core.mean(topRectangle).val[0];

/*
* Telemetry from on-field tests show that a rectangle sample on an orange ring produces a topAverage / bottomAverage
* of about 90, where no ring (no orange color) yields something around 125.  So, let's use 110 as the threshold,
* above 100 is no ring, below 100 is ring.
*/
            if (topAverage < 110 && bottomAverage < 110) {      // Both rectangles detect orange = 4 rings
                ringPosition = "C";
            } else if (topAverage > 110 && bottomAverage < 110) {   // Top rectangle is not orange, bottom
                ringPosition = "B";                                 // is orange = 1 ring
            } else {                    // default is no rings
                ringPosition = "A";
            }

//            topRectangle.release();
//            bottomRectangle.release();

            return input;
        }
    }
}
