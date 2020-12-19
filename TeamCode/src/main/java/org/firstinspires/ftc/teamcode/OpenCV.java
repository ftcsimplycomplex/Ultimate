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
    private String ringPosition;
    private OpenCvWebcam camera;

    /*
    * "Getter" method to return target zone A, B, or C as a string
    */
    public String getPosition()
    {
        return ringPosition;
    }

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


    class UltimatePipeline extends OpenCvPipeline {

        private Mat topRectangle = new Mat();
        private Mat bottomRectangle = new Mat();
        private Rect bottomRect = new Rect(
                360,
                440,
                40,
                10
        );
        private Rect topRect = new Rect(
                360,
                390,
                40,
                10
        );
        private double bottomAverage;
        private double topAverage;

        @Override
        public Mat processFrame(Mat input) {
            if (input.empty()) return input;

            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);

            Imgproc.rectangle(input, topRect, new Scalar(0, 255, 0), 2);
            Imgproc.rectangle(input, bottomRect, new Scalar(0, 255, 0), 2);

            Core.extractChannel(input.submat(bottomRect), bottomRectangle, 2);
            Core.extractChannel(input.submat(topRect), topRectangle, 2);

            bottomAverage = Core.mean(bottomRectangle).val[0];
            topAverage = Core.mean(topRectangle).val[0];

            if (topAverage < 50 && bottomAverage < 50) {
                ringPosition = "C";
            } else if (topAverage > 50 && bottomAverage < 50) {
                ringPosition = "B";
            } else {
                ringPosition = "A";
            }

//            topRectangle.release();
//            bottomRectangle.release();

            return input;
        }
    }
}
