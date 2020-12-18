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

public class OpenCV extends OpenCvPipeline {
    public String ringPosition;
    public Mat topRectangle = new Mat();
    public Mat bottomRectangle = new Mat();
    public OpenCvWebcam camera;

    public String getPosition()
    {
        return ringPosition;
    }
    public OpenCV(HardwareMap hardwareMap) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
    }

    public void init(OpenCV detector) {
        camera.openCameraDevice();
        camera.setPipeline(detector);
        camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
    }

    @Override
    public Mat processFrame(Mat input) {
        if(input.empty()) return input;

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);

        Rect bottomRect = new Rect(
                200,
                200,
                10,
                40
        );
        Rect topRect = new Rect(
                100,
                100,
                10,
                40
        );

        Imgproc.rectangle(input, topRect, new Scalar(0, 255, 0), 2);
        Imgproc.rectangle(input, bottomRect, new Scalar(0, 255, 0), 2);

        Core.extractChannel(input.submat(bottomRect), bottomRectangle, 2);
        Core.extractChannel(input.submat(topRect), topRectangle, 2);

        double bottomAverage = Core.mean(bottomRectangle).val[0];
        double topAverage = Core.mean(topRectangle).val[0];

        if(topAverage < 50 && bottomAverage < 50) {
            ringPosition = "C";
        } else if(topAverage > 50 && bottomAverage < 50) {
            ringPosition = "B";
        } else {
            ringPosition = "A";
        }


        return input;
    }
}
