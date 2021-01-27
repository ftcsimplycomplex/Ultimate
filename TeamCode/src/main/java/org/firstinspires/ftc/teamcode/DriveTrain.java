package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

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

    //Declaring motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    public DriveTrain(OpMode opMode){
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

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

        }
        // stops the robot
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);

        // resets the mode
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void rotate(int degrees, double speed){

        // set correct modes for motors
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // created targets
        int leftRearTarget;
        int rightRearTarget;
        int leftFrontTarget;
        int rightFrontTarget;

        // defined target variables
        // 53.41 is about the circumference of one rotation of our robot - 17/2 * pi - 17 inch is about the diameter of our robot
        leftFrontTarget = leftFront.getCurrentPosition() + (int)(62.24 * degrees * 1.45 * COUNTS_PER_INCH/ 360);
        rightFrontTarget = rightFront.getCurrentPosition() +(int)(-62.24 * degrees *1.45 * COUNTS_PER_INCH/ 360);
        leftRearTarget = leftRear.getCurrentPosition() + (int)(62.24 * degrees * 1.45* COUNTS_PER_INCH/ 360);
        rightRearTarget = rightRear.getCurrentPosition() + (int)(-62.24 * degrees * 1.45 * COUNTS_PER_INCH/ 360);

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

        }

        // robot stopped
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        // defined target variables
        // had diagonals go the same direction - 2 positive, 2 negative
        leftFrontTarget = leftFront.getCurrentPosition() + (int)(horizontalInches*COUNTS_PER_INCH * 1.05);
        rightFrontTarget = rightFront.getCurrentPosition() +(int)(-horizontalInches*COUNTS_PER_INCH * 1.05);
        leftRearTarget = leftRear.getCurrentPosition() + (int)(-horizontalInches*COUNTS_PER_INCH * 1.05);
        rightRearTarget = rightRear.getCurrentPosition() + (int)(horizontalInches*COUNTS_PER_INCH * 1.05);

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

        }

        // robot stops
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);

        // reset mode
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

