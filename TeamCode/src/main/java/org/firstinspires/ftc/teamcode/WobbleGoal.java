package org.firstinspires.ftc.teamcode;

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

public class WobbleGoal {
    //Declare Varibles For Contructing The Object
    private OpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    //Declare Varibles for motors
    private Servo clawServo;
    private Servo armServo;
    //Set Constants for All the Different Servo Postions
    private double OPEN_CLAW_POS = 0.0;
    private double CLOSED_CLAW_POS = 0.0;
    private double PARK_ARM_POS = 0.205;    //Reference engineering notebook for numbers 11/25/2020
    private double LIFT_ARM_POS = 0.08;     //Reference engineering notebook for numbers 11/25/2020
    private double GRAB_GOAL_POS = 0.03;    //Reference engineering notebook for numbers 11/25/2020

    public WobbleGoal(OpMode opMode){
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        //Find Servos
        clawServo = hardwareMap.get(Servo.class, "CLAW");
        armServo = hardwareMap.get(Servo.class, "ARM");
    }
    //Make Methods for all the different actions
    public void openClaw(){
        clawServo.setPosition(OPEN_CLAW_POS);
    }
    public void closeClaw(){
        clawServo.setPosition(CLOSED_CLAW_POS);
    }
    public void parkArm(){
        armServo.setPosition(PARK_ARM_POS);
    }
    public void liftArm(){
        armServo.setPosition(LIFT_ARM_POS);
    }
    public void grabGoal(){
        armServo.setPosition(GRAB_GOAL_POS);
    }
}
