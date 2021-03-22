package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 *  Class to hold static objects and methods for using the orientation sensor BNO055 in the REV Hub.
 *  Goal here is to initialize the sensor object and "zero" it at robot initialization, before
 *  Autonomous, and preserve the same zero angle orientation through TeleOp.
 *
 *
 */
public class OrientationUtil {

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
//    private OpMode opMode;                // Probably don't need this


    private static BNO055IMU imu;           // Only created once
    private static Orientation angles;      // Cribbed from Concept program
    private static Acceleration gravity;    // Cribbed from Concept program
    private static BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); // Cribbed from Concept program

    private static boolean isInitialized = false;   // Will be made true in the Constructor
    private static float baseOrientation;           // IMU reported orientation at initialization.  Probably zero
    private static float currentOrientation;        // updated when new orientation information is requested

    /**
     * Constructor
     * */
    public OrientationUtil(OpMode opMode){      // Call the constructor with "this" as the argument
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        // Retrieve the IMU.  We're only going to initialize it if it hasn't already been initialized.
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        if(!isInitialized) {
            resetImu();
            isInitialized = true;
        }
    }

    /**
     * resetImU() - reinitializes the IMU.  Expected to be called externally, from Autonomous OpMode to
     * zero the IMU during field setup.  Called internally if IMU is not already initialized.
     *
     */
    public void resetImu(){

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        baseOrientation = angles.firstAngle;
    }

    /**
     * isImuInitialized() - getter method to report whether or not the Constructor has been called.
     * Normally, the Autonomous OpMode calls the Constructor during init, and there's no need for the
     * TeleOp opMode to call it.  Having this method allows the TeleOp OpMode to determine if a Constructor
     * call is needed.
     *
     */
    public boolean isImuInitialized(){
        return isInitialized;
    }

    /** getCurrentOrientation() - getter method to read the IMU, and report current heading.
     *
     */
    public float getCurrentOrientation(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentOrientation = angles.firstAngle;
        return currentOrientation;
    }

    /**
     * getBaseOrientation() - getter method to return the base orientation determined at initialization,
     * in the Constructor.  This should probably always be zero, but best practice is to not assume that.
     *
     */
    public float getBaseOrientation() {
        return baseOrientation;
    }

}
