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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor frontMotor = null;
    private DcMotor backMotor = null;

    // Declare Buttons
    private boolean leftUpX;
    private boolean rightUpB;
    private boolean midUpA;
    private boolean yButton;
    private boolean lBumperUp;
    private boolean rBumperUp;
    private boolean dpadDownUp;
    private boolean dpadUpUp;


    // Declare Wobble Goal Mechanism
    public WobbleGoal wobbleGoal;
    // Declare kicker
    public Kicker kicker;
    // Declare Joystick variables
    private double translateY, translateX, rotate;
    // Declare motor power variables
    private double lfPower, lrPower, rfPower, rrPower;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        wobbleGoal = new WobbleGoal(this);
        kicker = new Kicker(this);

        leftFront  = hardwareMap.get(DcMotor.class, "LFD");
        rightFront = hardwareMap.get(DcMotor.class, "RFD");
        leftRear  = hardwareMap.get(DcMotor.class, "LRD");
        rightRear = hardwareMap.get(DcMotor.class, "RRD");
        frontMotor = hardwareMap.get(DcMotor.class, "GECKO"); //Front intake motor
        backMotor = hardwareMap.get(DcMotor.class, "COMPLIANT"); //Back intake motor
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        //Start Positions for Servos
        wobbleGoal.parkArm();
        wobbleGoal.closeClaw();

        //start position for ring drop arm
        wobbleGoal.raiseRings();

        //Start position for kicker
        kicker.rest();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

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
            if (!gamepad2.dpad_down){
                dpadDownUp = true;
            }
            if (!gamepad2.dpad_up){
                dpadUpUp = true;
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
            if(gamepad2.left_bumper && lBumperUp) {
            kicker.shoot();
            sleep(500);
            kicker.rest();
            }

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
             //Calculating power needed in each motor
            //turtleMode - 50% of normal power - if Right bumper is pressed (not toggle) turtleMode is on, if not, normal mode
            if (gamepad1.right_bumper) {
                lfPower = (translateY + translateX + rotate)/2;
                lrPower = (translateY - translateX + rotate)/2;
                rfPower = (translateY - translateX - rotate)/2;
                rrPower = (translateY + translateX - rotate)/2;
            } else {
                lfPower = translateY + translateX + rotate;
                lrPower = translateY - translateX + rotate;
                rfPower = translateY - translateX - rotate;
                rrPower = translateY + translateX - rotate;
            }

            // Send calculated power to wheels
            leftFront.setPower(Range.clip(lfPower, -1.0, +1.0));
            leftRear.setPower(Range.clip(lrPower, -1.0, +1.0));
            rightFront.setPower(Range.clip(rfPower, -1.0, +1.0));
            rightRear.setPower(Range.clip(rrPower, -1.0, +1.0));

            // Show the elapsed game time and wheel power.
           /* telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", lfPower, rfPower);
            telemetry.update();
        */
        }
    }
}
