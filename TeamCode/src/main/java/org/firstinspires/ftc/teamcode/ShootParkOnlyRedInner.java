
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="ShootParkOnlyRedInner", group="Red")
/*@Disabled*/
public class ShootParkOnlyRedInner extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;    // eg: Gobilda 5202-0002-0014 with a 13.7:1 gear box
    static final double     DRIVE_GEAR_REDUCTION    = 2.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.75;
    static final double     TURN_SPEED              = 0.5;


    public WobbleGoal wobbleGoal; 
    public DriveTrain driveTrain;
    public Kicker kicker;

    public String ringPosition;
    public OpenCV detector;

    public float initialAngle;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "INIT");
        telemetry.update();
        wobbleGoal = new WobbleGoal(this);
        driveTrain = new DriveTrain (this);
        kicker = new Kicker (this);

        //Set start position when init is pressed
        wobbleGoal.parkArm();
        wobbleGoal.closeClaw();
        kicker.rest();


        ringPosition = "B";         // Default if we have to comment out vision

        detector = new OpenCV(hardwareMap);
        telemetry.addData("Status", "WAITING");
        telemetry.update();
//        detector.init(detector);
        while(!opModeIsActive()){
            ringPosition = detector.getPosition();
            telemetry.addData("topAverage: ",detector.getTopAverage());
            telemetry.addData("bottomAverage: ",detector.getBottomAverage());
            telemetry.addData("Target Zone is ", ringPosition);
            telemetry.update();
            sleep(250);     // Report four times per second
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        detector.stopDetect();      // Shut down webcam processing

        kicker.setFlywheelSpeed(0.44);
        sleep(3500);
        driveTrain.controlledTankDrive(DRIVE_SPEED, -2, -2);
        driveTrain.controlledStraffe(12, DRIVE_SPEED);
        kicker.flywheel();
        driveTrain.rotate(-5, DRIVE_SPEED);
        sleep(2000); // Start shooting preloaded rings
        kicker.shoot();
        sleep(150);
        kicker.rest();
        sleep(2000);
        kicker.shoot();
        sleep(150);
        kicker.rest();
        sleep(2000);
        kicker.shoot();
        sleep(150);
        /*kicker.rest();
        sleep( 2500);
        kicker.shoot();
        sleep(150);*/
        kicker.rest();
        kicker.stopFlywheel();
        driveTrain.rotate(5, DriveTrain.DRIVE_SPEED);
        sleep(6500);
        driveTrain.controlledTankDrive(DRIVE_SPEED,-72, -72);
        driveTrain.rotate(180, DRIVE_SPEED);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public void WobbleA(){
        driveTrain.tankDrive(DRIVE_SPEED,  -5,  -5);  // S1: Forward to zone A
        driveTrain.straffe(16.0, 0.5);
        wobbleGoal.grabGoal(); // S2: Lower Wobble Goal
        sleep(3000);     // pause for servos to move
        wobbleGoal.openClaw(); // S3: Let go of Wobble Goal
        sleep(1200); //pause for servos to move
        driveTrain.straffe(2.0,0.2); //S4: Strafe to the left 2 inches
        sleep(1100);     // pause for servos to move
        wobbleGoal.parkArm(); // S5: Raise arm
        sleep(1200);     // pause for servos to move
        driveTrain.straffe(-28.0, 0.5);
        driveTrain.tankDrive(DRIVE_SPEED,  -14.0,  -14.0); // S5: Back up to Launch Line
        driveTrain.rotate(180, 0.5);
    }
    public void WobbleB(){
        kicker.startIntake();
        driveTrain.controlledStraffe(-12,0.5);
        sleep(500);
        driveTrain.fixAngle(initialAngle);
        driveTrain.tankDrive(0.4,18,18);
        sleep(1000);
        driveTrain.tankDrive(0.4,-18,-18);
        kicker.flywheel();
        driveTrain.controlledStraffe(13.67,0.5);
        kicker.stopIntake();
        sleep(300);
        driveTrain.fixAngle(initialAngle);
        sleep(300);
        kicker.shoot();
        sleep(150);
        kicker.rest();
        sleep(300);
        kicker.shoot();
        sleep(150);
        kicker.rest();
        kicker.stopFlywheel();
        driveTrain.controlledTankDrive(DRIVE_SPEED,-24.6,-24.6);  // S1: Forward to zone B
        driveTrain.straffe(2, 0.5);
        wobbleGoal.grabGoal(); // S2: Lower Wobble Goal
        sleep(3000);     // pause for servos to move
        wobbleGoal.openClaw(); // S3: Let go of Wobble Goal
        sleep(1200); //pause for servos to move
        driveTrain.straffe(2.0,0.2); //S4: Strafe to the left 2 inches
        sleep(1100);     // pause for servos to move
        wobbleGoal.parkArm(); // S5: Raise arm
        sleep(1200);     // pause for servos to move
        driveTrain.tankDrive(DRIVE_SPEED,  9,  9); // S5: Back up to Launch Line
        driveTrain.rotate(180, 0.5);
    }
    public void WobbleC(){
        driveTrain.controlledStraffe(-10.68,0.5);
        sleep(500);
        driveTrain.fixAngle(initialAngle);
        driveTrain.tankDrive(0.76,16,16); // Knocks over starter stack
        sleep(500);
        driveTrain.fixAngle(initialAngle);
        driveTrain.tankDrive(0.76,-9,-9); // Backs up from starter stack
        kicker.startIntake();
        driveTrain.tankDrive(0.3, 15, 15); // Intakes starter stack
        sleep(500);
        driveTrain.controlledTankDrive(0.76, -22, -22); // Backs up from starter stack
        kicker.flywheel();
        driveTrain.controlledStraffe(13.43,0.5); // Moves to shooting pos
        sleep(300);
        kicker.stopIntake();
        driveTrain.fixAngle(initialAngle);
        sleep(300); // Starter Stack shooting
        kicker.shoot();
        sleep(150);
        kicker.rest();
        sleep(200);
        kicker.shoot();
        sleep(150);
        kicker.rest();
        sleep(200);
        kicker.shoot();
        sleep(150);
        kicker.rest();
        sleep(200);
        kicker.shoot();
        sleep(150);
        kicker.rest();
        sleep(200);
        kicker.shoot();
        sleep(150);
        kicker.rest();
        sleep(200);
        kicker.shoot();
        sleep(150);
        kicker.rest();
        kicker.stopFlywheel(); // After shooting start stack
        driveTrain.controlledTankDrive(0.70,  -46,  -46);  // S1: Forward to zone C
        driveTrain.straffe(15.99, 0.76);
        wobbleGoal.grabGoal(); // S2: Lower Wobble Goal
        sleep(3000);     // pause for servos to move
        wobbleGoal.openClaw(); // S3: Let go of Wobble Goal
        sleep(760); //pause for servos to move
        //sleep(1100);     // pause for servos to move
        wobbleGoal.parkArm(); // S5: Raise arm
        //sleep(1200);     // pause for servos to move
        driveTrain.controlledTankDrive(0.70,  33.0,  33.0); // S5: Back up to Launch Line
    }
}



