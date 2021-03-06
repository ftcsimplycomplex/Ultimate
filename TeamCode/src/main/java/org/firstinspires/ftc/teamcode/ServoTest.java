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
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Interactive program to allow manual positioning of two servos, reporting via on-screen telemetry
 * the servo position.  Intended to be used to determine positions to be hard-coded into robot OpModes.
 *
 */
@TeleOp(name="ServoTest", group="Linear Opmode")
//@Disabled
public class ServoTest extends LinearOpMode {

    private Servo RightIntake;
    private Servo LeftIntake;

    private boolean xUp, yUp, aUp, bUp;

    private double RightPos = 0.30;
    private double LeftPos = 0.79;

    @Override
    public void runOpMode() {
        //Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        RightIntake = hardwareMap.get(Servo.class, "ARM");
       LeftIntake = hardwareMap.get(Servo.class, "CLAW");

        telemetry.addLine("Servos mapped");
        telemetry.update();

        waitForStart();
/*
        RightIntake.setPosition(RightPos);
        LeftIntake.setPosition(LeftPos);
*/

        while (opModeIsActive()){

            if(!gamepad1.a)
                aUp = true;
            if(!gamepad1.b)
                bUp = true;
            if(!gamepad1.x)
                xUp = true;
            if(!gamepad1.y)
                yUp = true;

            if(gamepad1.x && xUp){
                // increases the positional value of the right intake servo
                xUp = false;
                RightPos += 0.01;
                RightIntake.setPosition(RightPos);

                // add telemetry
                telemetry.addData("Right: ","%.03f", RightPos);
                telemetry.addData("Left: ","%.03f", LeftPos);

                // update telemetry
                telemetry.update();
            }
            if(gamepad1.y && yUp){
                // decreases the positional value of the right intake servo
                yUp = false;
                RightPos -= 0.01;
                RightIntake.setPosition(RightPos);

                // add telemetry
                telemetry.addData("Right: ","%.03f", RightPos);
                telemetry.addData("Left: ","%.03f", LeftPos);

                // update telemetry
                telemetry.update();
            }
            if(gamepad1.a && aUp){
                // increases the positional value of the left intake servo
                aUp = false;
                LeftPos += 0.01;
                LeftIntake.setPosition(LeftPos);

                // add telemetry
                telemetry.addData("Right: ","%.03f", RightPos);
                telemetry.addData("Left: ","%.03f", LeftPos);

                // update telemetry
                telemetry.update();
            }
           if(gamepad1.b && bUp){
                // decreases the positional value of the left intake servo
                bUp = false;
                LeftPos -= 0.01;
                LeftIntake.setPosition(LeftPos);
                // add telemetry
                telemetry.addData("Right: ","%.03f", RightPos);
                telemetry.addData("Left: ","%.03f", LeftPos);

                // update telemetry
                telemetry.update();
            }
        }
    }
}
