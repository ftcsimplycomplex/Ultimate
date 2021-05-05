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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

/*Modifying sample code to support 4 motors*/

@TeleOp(name="Mecanum Drive", group="Linear Opmode")
//@Disabled
public class MecanumDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    // Declare motors
    private DcMotor frontMotor = null;
    private DcMotor backMotor = null;

    // Declare Buttons
    private boolean leftUpX;
    private boolean rightUpB;
    private boolean midUpA;
    private boolean yButton;
    private boolean lBumperUp;
    private boolean lBumperUpGP1;
    private boolean rBumperUp;
    private boolean dpadDownUp;
    private boolean dpadUpUp;
    private boolean dpadLeftUp;
    private boolean gp1aUP;
    private boolean gp1xUp;
    private boolean gp1bUp;
    private boolean gp1yUp;

    // Declare initial angle
    private float initialAngle;

    // Declare Wobble Goal Mechanism
    public WobbleGoal wobbleGoal;

    // Declare kicker
    public Kicker kicker;

    // Declare driveTrain
    public DriveTrain driveTrain;

    // Declare Joystick variables
    private double translateY, translateX, rotate;

    // Declare Constant variable to make it possible to toggle direction for gamepad controls
    private double sensitivty = 1;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize objects
        wobbleGoal = new WobbleGoal(this);
        kicker = new Kicker(this);
        driveTrain = new DriveTrain(this);


        frontMotor = hardwareMap.get(DcMotor.class, "GECKO"); //Front intake motor
        backMotor = hardwareMap.get(DcMotor.class, "COMPLIANT"); //Back intake motor

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Start Positions for Servos
        wobbleGoal.parkArm();
        wobbleGoal.closeClaw();

        //start position for ring drop arm
        wobbleGoal.raiseRings();

        //Start position for kicker
        kicker.rest();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        kicker.setFlywheelPIDF();
        runtime.reset();

        // Set initialAngle
        initialAngle = driveTrain.readAngle();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Checking for button press
           if(!gamepad2.a){
               midUpA = true;
           }
            if(!gamepad2.b){
                rightUpB = true;
            }
            if(!gamepad2.x){
                leftUpX = true;
            }
            if(!gamepad2.y){
                yButton = true;
            }
            if(!gamepad2.left_bumper){
                lBumperUp = true;
            }
            if (!gamepad2.right_bumper) {
                rBumperUp = true;
            }
            if (!gamepad2.dpad_down){   // dpad down reduces speed of flywheel
                dpadDownUp = true;
            }
            if (!gamepad2.dpad_up){     // dpad up increases speed of flywheel
                dpadUpUp = true;
            }
            if (!gamepad2.dpad_left){     // dpad left reports speed of flywheel
                dpadLeftUp = true;
            }
            if(!gamepad1.left_bumper){
                lBumperUpGP1 = true;
            }

            if(!gamepad1.a){
                gp1aUP = true;
            }

            if(!gamepad1.x){
                gp1xUp = true;
            }

            if(!gamepad1.b){
                gp1bUp = true;
            }

            if(!gamepad1.y){
                gp1yUp = true;
            }


            // gamepad button X - Park Arm
            if(gamepad2.x && leftUpX){
                leftUpX = false;
                wobbleGoal.parkArm();
            }

            // gamepad button A - Lift Arm
            if(gamepad2.a && midUpA){
                midUpA = false;
                wobbleGoal.liftArm();
                //Changes servo to liftArm position
            }

            // gamepad button B - Grab Goal Position
            if(gamepad2.b && rightUpB){
                rightUpB = false;
                wobbleGoal.grabGoal();
            }
            // gamepad button Y - Toggles Claw
            if(gamepad2.y && yButton) {
                yButton = false;
                // toggles claw
                if (wobbleGoal.clawOpen) {
                    wobbleGoal.closeClaw();
                } else {
                    wobbleGoal.openClaw();
                }
            }

            // Shoots kicker
            if(gamepad2.left_bumper && lBumperUp) {
                kicker.shoot();
                sleep(150);
                kicker.rest();
            }

            // Starts flywheel
            if (gamepad2.right_bumper && rBumperUp) {
                kicker.flywheel();
            }
            else {
                kicker.stopFlywheel();
            }

            // Change the flywheel speed: Dpad buttons set the speed invoked by the right bumper
            if(gamepad2.dpad_up && dpadUpUp){
                dpadUpUp = false;
                kicker.increaseFlywheel();
            }
            if(gamepad2.dpad_down && dpadDownUp){
                dpadDownUp = false;
                kicker.decreaseFlywheel();
            }

            if(gamepad2.dpad_left && dpadLeftUp){   // Reports flywheel actual velocity
                dpadLeftUp = false;
                kicker.reportFlywheelVelocity();
            }

            // Turns robot to face initial angle
            if(gamepad1.a && gp1aUP){
                driveTrain.fixAngle(initialAngle);
                gp1aUP = false;
            }

            // Strafes for power shots
            if(gamepad1.x && gp1xUp){
                driveTrain.straffe(7,0.6);
                sleep(200);
                driveTrain.fixAngle(initialAngle);
                gp1xUp = false;
            }

            // Sets initialAngle
            if(gamepad1.b && gp1bUp){
                initialAngle = driveTrain.readAngle();
                gp1bUp = false;
            }

            // Controlled strafe for power shots
            if(gamepad1.y && gp1yUp) {
                driveTrain.controlledStraffe(7, 0.6);
                sleep(200);
                driveTrain.fixAngle(initialAngle);
                gp1xUp = false;
            }

            //using left and right trigger for intake
            if (gamepad2.left_trigger!=0) {
                frontMotor.setPower(1);
                backMotor.setPower(1);

                //added telemetry
                telemetry.addData("trigger values","left (%.2f), right (%.2f)",gamepad2.left_trigger,gamepad2.right_trigger);
                telemetry.update();
            } else {
                if (gamepad2.right_trigger!=0) {
                    frontMotor.setPower(-1);
                    backMotor.setPower(-1);

                    //added telemetry
                    telemetry.addData("trigger values","left (%.2f), right (%.2f)",gamepad2.left_trigger,gamepad2.right_trigger);
                    telemetry.update();
                } else {
                    frontMotor.setPower(0);
                    backMotor.setPower(0);
                }
            }


             //defining joystick variables
             translateY  = -gamepad1.left_stick_y ;
             rotate = gamepad1.right_stick_x ;
             translateX = gamepad1.left_stick_x;

             // Toggles reverse direction
             if(gamepad1.left_bumper&&lBumperUpGP1){
                 lBumperUpGP1=false;
                 if(sensitivty==1){
                     sensitivty=-1;
                 }else{
                     sensitivty=1;
                 }
             }

             // Calculates and sets motor powers
             driveTrain.driverControlled(translateX, translateY,rotate,sensitivty,gamepad1.right_bumper);

        }
    }
}

