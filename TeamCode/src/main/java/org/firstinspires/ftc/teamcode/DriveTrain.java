package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;

// We made it a class so it was easier to do different motions Ex. driving, straffing, turning, braking
/*
 Instructions:
 tankDrive(leftInches, rightInches, speed)
    leftInches is for both the left front and left back motor
    rightInches is for both the right front and right back motor
    speed has to be a positive double from 0.0 to 1.0

 rotate(degrees, speed)
    A positive turn is clockwise while negative is counter clockwise
    degrees can be any integer, positive or negative
    speed has to be a positive double from 0.0 to 1.0

 straffe(horizantalInches, speed)
    straffe only goes left or right
    speed has to be a positive double from 0.0 to 1.0
  stop()
    self explanatory

   ALL of these other then stop() work using encoders
*/
public class DriveTrain {

    private OpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;    // eg: Gobilda 5202-0002-0014 with a 13.7:1 gear box
    static final double     DRIVE_GEAR_REDUCTION    = 2.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.5;
    static final float K_PROP_TD = 0.05f;
    static final float K_PROP_S = 0.035f;
    static final float K_PROP_R = (1.0f/360.0f);

    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;


    //Declaring motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    public DriveTrain(OpMode opMode){
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "LFD");
        rightFront = hardwareMap.get(DcMotor.class, "RFD");
        leftRear  = hardwareMap.get(DcMotor.class, "LRD");
        rightRear = hardwareMap.get(DcMotor.class, "RRD");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void stop(){
        // sets motor power to 0 to stop robot
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);
    }


    public void tankDrive(double speed, double rightInches, double leftInches){
        // set motors to correct mode
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // made target variables
        int leftRearTarget;
        int rightRearTarget;
        int leftFrontTarget;
        int rightFrontTarget;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float error;
        float targetAngle;
        targetAngle = angles.firstAngle;
        double rotVal = 0;

        // defined target variables
        leftFrontTarget = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        rightFrontTarget = rightFront.getCurrentPosition() +(int)(rightInches * COUNTS_PER_INCH);
        leftRearTarget = leftRear.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        rightRearTarget = rightRear.getCurrentPosition() + (int)( rightInches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftRear.setTargetPosition(leftRearTarget);
        rightRear.setTargetPosition(rightRearTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        leftFront.setPower(Range.clip(Math.abs(speed),-1,1));
        rightFront.setPower(Range.clip(Math.abs(speed),-1,1));
        leftRear.setPower(Range.clip(Math.abs(speed),-1,1));
        rightRear.setPower(Range.clip(Math.abs(speed),-1,1));

        // keep looping while we are still active, and there is time left, and all 4 motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when any of the motors hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that ALL motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

        while (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
           error = angles.firstAngle - targetAngle;

            // fix the angleError so it doesn't go past 180 degrees
            if (Math.abs(error) > 180.0) {
                error = -Math.copySign(360.0f - Math.abs(error), error);
            }
            rotVal = error * K_PROP_TD;

            if (rightInches < 0) { //This makes the robot's error negative when driving backwards
                rotVal = rotVal * (-1);
            }

            // Sets power
            leftFront.setPower (speed + rotVal);
            leftRear.setPower (speed + rotVal);
            rightFront.setPower (speed - rotVal);
            rightRear.setPower (speed - rotVal);

        }
        // stops the robot
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);

        //IMU
        angles= imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        error = angles.firstAngle - targetAngle;
        rotate (-error, speed);

        // resets the mode
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void rotate(float degrees, double speed){

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float error;
        float targetAngle;
        targetAngle = angles.firstAngle + degrees;

        error = angles.firstAngle-targetAngle;

        // Fixes errors more than 180
        if (Math.abs(error) > 180.0) {
            error = -Math.copySign(360.0f - Math.abs(error), error);
        }

        double rotVal = 0;

        // set correct modes for motors
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Sets brake mode to reduce coasting
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (Math.abs(error) > 1){

            angles= imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = angles.firstAngle-targetAngle;

            // checks to see if error is more than 180
            if (Math.abs(error) > 180.0) {
                error = -Math.copySign(360.0f - Math.abs(error), error);
            }

            // sets rotVal
            rotVal = Math.abs(error * K_PROP_R);


            // checks which direction your turning
            if (error < 0) {
                leftFront.setPower( - (rotVal + speed));
                leftRear.setPower( - (rotVal + speed));
                rightFront.setPower( + rotVal + speed);
                rightRear.setPower( + rotVal + speed);
            } else {
                leftFront.setPower( + rotVal + speed);
                leftRear.setPower( + rotVal + speed);
                rightFront.setPower( - (rotVal + speed));
                rightRear.setPower( - (rotVal + speed));
            }
        }

        // stops motors
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);

        // sets correct mode
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // sets float mode
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    //Input a negative horizontalInches to go left, positive to go right
    public void straffe(double horizontalInches, double speed){

        // set correct modes for motors
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Created target variables
        int leftRearTarget;
        int rightRearTarget;
        int leftFrontTarget;
        int rightFrontTarget;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float error;
        float targetAngle;
        targetAngle = angles.firstAngle;
        double rotVal = 0;

        // defined target variables
        // had diagonals go the same direction - 2 positive, 2 negative
        leftFrontTarget = leftFront.getCurrentPosition() + (int)(horizontalInches*COUNTS_PER_INCH * 1.05);
        rightFrontTarget = rightFront.getCurrentPosition() +(int)(-horizontalInches*COUNTS_PER_INCH * 1.05);
        leftRearTarget = leftRear.getCurrentPosition() + (int)(-horizontalInches*COUNTS_PER_INCH * 1.05);
        rightRearTarget = rightRear.getCurrentPosition() + (int)(horizontalInches*COUNTS_PER_INCH * 1.05);

        // Sets target positions
        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftRear.setTargetPosition(leftRearTarget);
        rightRear.setTargetPosition(rightRearTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        leftFront.setPower(Range.clip(Math.abs(speed),-1,1));
        rightFront.setPower(Range.clip(Math.abs(speed),-1,1));
        leftRear.setPower(Range.clip(Math.abs(speed),-1,1));
        rightRear.setPower(Range.clip(Math.abs(speed),-1,1));

        // keep looping while we are still active, and there is time left, and all 4 motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when any of the motors hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that ALL motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = angles.firstAngle - targetAngle;

            // fix the angleError so it doesn't go past 180 degrees
            if (Math.abs(error) > 180.0) {
                error = -Math.copySign(360.0f - Math.abs(error), error);
            }

            rotVal = error * K_PROP_S;

            if (horizontalInches < 0) { //This makes the robot's error negative when driving backwards
                rotVal = rotVal * (-1);
            }

            // Sets power
            leftFront.setPower (speed + rotVal);
            leftRear.setPower (speed + rotVal);
            rightFront.setPower (speed - rotVal);
            rightRear.setPower (speed - rotVal);

        }

        // robot stops
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);

        angles= imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        error = angles.firstAngle - targetAngle;
        rotate (-error, speed);

        // reset mode
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void controlledTankDrive(double speed, double rightInches, double leftInches){
        // set motors to correct mode
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // made target variables
        int leftRearTarget;
        int rightRearTarget;
        int leftFrontTarget;
        int rightFrontTarget;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float error;
        float targetAngle;
        targetAngle = angles.firstAngle;
        double rotVal = 0;
        int initialPos;
        int progress;
        int totalTicks;

        // Sets initial position- motor ticks
        initialPos = leftFront.getCurrentPosition();

        // defined target variables
        leftFrontTarget = initialPos + (int)(leftInches * COUNTS_PER_INCH);
        rightFrontTarget = rightFront.getCurrentPosition() +(int)(rightInches * COUNTS_PER_INCH);
        leftRearTarget = leftRear.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        rightRearTarget = rightRear.getCurrentPosition() + (int)( rightInches * COUNTS_PER_INCH);

        // Sets target positions
        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftRear.setTargetPosition(leftRearTarget);
        rightRear.setTargetPosition(rightRearTarget);

        // defines total ticks
        totalTicks = Math.abs(leftFrontTarget - initialPos);
        progress = 0;

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        leftFront.setPower(Range.clip(Math.abs(speed * 0.5),-1,1));
        rightFront.setPower(Range.clip(Math.abs(speed * 0.5),-1,1));
        leftRear.setPower(Range.clip(Math.abs(speed * 0.5),-1,1));
        rightRear.setPower(Range.clip(Math.abs(speed * 0.5),-1,1));

        // keep looping while we are still active, and there is time left, and all 4 motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when any of the motors hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that ALL motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = angles.firstAngle - targetAngle;

            // fix the angleError so it doesn't go past 180 degrees
            if (Math.abs(error) > 180.0) {
                error = -Math.copySign(360.0f - Math.abs(error), error);
            }
            rotVal = error * K_PROP_TD;

            if (rightInches < 0) { //This makes the robot's error negative when driving backwards
                rotVal = rotVal * (-1);
            }


       //     double percentage = ((leftFront.getCurrentPosition() + rightFront.getCurrentPosition() +
              //      rightRear.getCurrentPosition() + leftRear.getCurrentPosition())/(4 * leftFrontTarget)) * 100;

            progress = Math.abs(leftFront.getCurrentPosition() - initialPos);
            double percentage = (progress * 100.0)/totalTicks;

            // checks if in the first/last 5% of drive and makes speed half of what it was
            if (percentage < 5.0 || percentage > 95.0) {
                leftFront.setPower ((speed + rotVal) * 0.5);
                leftRear.setPower ((speed + rotVal) * 0.5);
                rightFront.setPower ((speed - rotVal) * 0.5);
                rightRear.setPower ((speed - rotVal) * 0.5);

            } else {
                leftFront.setPower (speed + rotVal);
                leftRear.setPower (speed + rotVal);
                rightFront.setPower (speed - rotVal);
                rightRear.setPower (speed - rotVal);
            }
        }


        // stops the robot
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        error = angles.firstAngle - targetAngle;
        rotate (Math.round(-error), speed);

        // resets the mode
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // returns heading
    public float readAngle() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle);
    }

    // returns robot to initialAngle
    public void fixAngle(float initialAngle){
        float currentAngle, error ;
        currentAngle = readAngle();
        error = currentAngle - initialAngle;
        rotate(-error, 0.5);
    }

    public void controlledStraffe(double horizontalInches, double speed) {

        // set correct modes for motors
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Created target variables
        int leftRearTarget;
        int rightRearTarget;
        int leftFrontTarget;
        int rightFrontTarget;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float error;
        float targetAngle;
        targetAngle = angles.firstAngle;
        double rotVal = 0;
        int initialPos;
        int progress;
        int totalTicks;

        // Sets initialPos
        initialPos = leftFront.getCurrentPosition();


        // defined target variables
        // had diagonals go the same direction - 2 positive, 2 negative
        leftFrontTarget = leftFront.getCurrentPosition() + (int) (horizontalInches * COUNTS_PER_INCH * 1.05);
        rightFrontTarget = rightFront.getCurrentPosition() + (int) (-horizontalInches * COUNTS_PER_INCH * 1.05);
        leftRearTarget = leftRear.getCurrentPosition() + (int) (-horizontalInches * COUNTS_PER_INCH * 1.05);
        rightRearTarget = rightRear.getCurrentPosition() + (int) (horizontalInches * COUNTS_PER_INCH * 1.05);

        // Sets target positions
        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftRear.setTargetPosition(leftRearTarget);
        rightRear.setTargetPosition(rightRearTarget);

        // Sets total ticks
        totalTicks = Math.abs(leftFrontTarget - initialPos);
        progress = 0;

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        leftFront.setPower(Range.clip(Math.abs(speed), -1, 1));
        rightFront.setPower(Range.clip(Math.abs(speed), -1, 1));
        leftRear.setPower(Range.clip(Math.abs(speed), -1, 1));
        rightRear.setPower(Range.clip(Math.abs(speed), -1, 1));

        // keep looping while we are still active, and there is time left, and all 4 motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when any of the motors hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that ALL motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = angles.firstAngle - targetAngle;

            // fix the angleError so it doesn't go past 180 degrees
            if (Math.abs(error) > 180.0) {
                error = -Math.copySign(360.0f - Math.abs(error), error);
            }

            // sets rotVal
            rotVal = error * K_PROP_S;

            if (horizontalInches < 0) { //This makes the robot's error negative when driving backwards
                rotVal = rotVal * (-1);
            }

            // sets progress
            progress = Math.abs(leftFront.getCurrentPosition() - initialPos);
            double percentage = (progress * 100.0) / totalTicks;

            // in the first and last 5% of drive, makes speed half of what it was
            if (percentage < 5.0 || percentage > 95.0) {
                leftFront.setPower((speed + rotVal) * 0.5);
                leftRear.setPower((speed + rotVal) * 0.5);
                rightFront.setPower((speed - rotVal) * 0.5);
                rightRear.setPower((speed - rotVal) * 0.5);

            } else {
                leftFront.setPower(speed + rotVal);
                leftRear.setPower(speed + rotVal);
                rightFront.setPower(speed - rotVal);
                rightRear.setPower(speed - rotVal);
            }
        }

        // robot stops
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        error = angles.firstAngle - targetAngle;
        rotate(-error, speed);

        // reset mode
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // calculates and sets motor powers for driver control
    public void driverControlled(double translateX, double translateY, double rotate, double sensitivty, boolean turtleMode ){

        // sets motor powers
       double lfPower, lrPower, rfPower, rrPower;

       // checks if in turtle mode or not
        if (turtleMode) {
            lfPower = sensitivty*(translateY + translateX + rotate)/2;
            lrPower = sensitivty*(translateY - translateX + rotate)/2;
            rfPower = sensitivty*(translateY - translateX - rotate)/2;
            rrPower = sensitivty*(translateY + translateX - rotate)/2;
        } else {
            lfPower = sensitivty*(translateY + translateX + rotate);
            lrPower = sensitivty*(translateY - translateX + rotate);
            rfPower = sensitivty*(translateY - translateX - rotate);
            rrPower = sensitivty*(translateY + translateX - rotate);
        }

        // makes sure speed is between -1 and 1
        leftFront.setPower(Range.clip(lfPower, -1.0, +1.0));
        leftRear.setPower(Range.clip(lrPower, -1.0, +1.0));
        rightFront.setPower(Range.clip(rfPower, -1.0, +1.0));
        rightRear.setPower(Range.clip(rrPower, -1.0, +1.0));

    }

    public void controlledRotate(float degrees, double speed){

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float error;
        float targetAngle;
        float initialAngle = readAngle();
        float percentage;
        targetAngle = angles.firstAngle + degrees;

        // sets error
        error = angles.firstAngle-targetAngle;

        // makes error more than 180
        if (Math.abs(error) > 180.0) {
            error = -Math.copySign(360.0f - Math.abs(error), error);
        }


        // set correct modes for motors
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // made it brake mode
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (Math.abs(error) > 1){

            angles= imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = angles.firstAngle-targetAngle;

            // makes sure error is more than 180
            if (Math.abs(error) > 180.0) {
                error = -Math.copySign(360.0f - Math.abs(error), error);
            }

            // calculates percentage
            percentage = (1-(error)/(targetAngle-initialAngle))*100;

            // if percentage is less than 5, speed is gradually increased
            if (percentage < 5.0) {

                if (error < 0) {
                    leftFront.setPower( - (speed + percentage/10));
                    leftRear.setPower( - (speed + percentage/10));
                    rightFront.setPower(speed + percentage/10);
                    rightRear.setPower(speed + percentage/10);
                } else {
                    leftFront.setPower(speed + percentage/10);
                    leftRear.setPower(speed + percentage/10);
                    rightFront.setPower( - (speed + percentage/10));
                    rightRear.setPower( - (speed + percentage/10));
                }

                // if percentage is more than 95, speed is gradually decreased
            } else if(percentage > 95){

                if (error < 0) {
                    leftFront.setPower( - (speed + (100 - percentage)/10));
                    leftRear.setPower( - (speed + (100 - percentage)/10));
                    rightFront.setPower(speed + (100 - percentage)/10);
                    rightRear.setPower(speed + (100 - percentage)/10);
                } else {
                    leftFront.setPower(speed  + (100 - percentage)/10);
                    leftRear.setPower(speed + (100 - percentage)/10);
                    rightFront.setPower( - (speed) + (100 - percentage)/10);
                    rightRear.setPower( - (speed) + (100 - percentage)/10);
                }

                // if percentage is between is between 5 and 95, keep consistent speed
            } else {
                if (error < 0) {
                    leftFront.setPower( - (speed + 0.5));
                    leftRear.setPower( - speed  + 0.5);
                    rightFront.setPower(speed + 0.5);
                    rightRear.setPower(speed + 0.5);
                } else {
                    leftFront.setPower(speed + 0.5);
                    leftRear.setPower(speed + 0.5);
                    rightFront.setPower( - (speed + 0.5));
                    rightRear.setPower( - (speed + 0.5));
                }
            }
        }

        // stops motors
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);

        //sets mode to run using encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // sets mode to float mode
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}



