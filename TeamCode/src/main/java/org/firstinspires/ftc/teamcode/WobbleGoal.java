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
    //Declare Variables For Constructing The Object
    private OpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    //Declare Variables for motors
    private Servo clawServo;
    private Servo armServo;
    private Servo ringServo;
    //Set Constants for All the Different Servo Positions
    private double OPEN_CLAW_POS = 0.4;     //Reference engineering notebook for numbers 11/28/2020
    private double CLOSED_CLAW_POS = 0.7;   //Reference engineering notebook for numbers 11/28/2020
/*
    private double PARK_ARM_POS = 0.205;    //Reference engineering notebook for numbers 11/25/2020
    private double LIFT_ARM_POS = 0.08;     //Reference engineering notebook for numbers 11/25/2020
    private double GRAB_GOAL_POS = 0.03;    //Reference engineering notebook for numbers 11/25/2020
*/
    private double PARK_ARM_POS = 0.61;     // Revised 10-JAN-2021 by JL & KER
    private double LIFT_ARM_POS = 0.49;     // Revised 10-JAN-2021 by JL & KER
    private double GRAB_GOAL_POS = 0.435;   // Revised 10-JAN-2021 by JL & KER

    private double DROP_RINGS_POS = 0.08;
    private double RAISE_ARM2_POS = 0.83;
    //Set boolean to store whether claw is open or closed
    public boolean clawOpen;
    public boolean arm2Up;

    public WobbleGoal(OpMode opMode){
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        //Find Servos
        clawServo = hardwareMap.get(Servo.class, "CLAW");
        armServo = hardwareMap.get(Servo.class, "ARM");
        ringServo = hardwareMap.get(Servo.class,"RING");

    }
    //Make Methods for all the different actions
    public void openClaw(){
        clawServo.setPosition(OPEN_CLAW_POS);
        clawOpen = true;
    }
    public void closeClaw(){
        clawServo.setPosition(CLOSED_CLAW_POS);
        clawOpen = false;
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

    public void dropRings(){ringServo.setPosition(DROP_RINGS_POS);arm2Up = false;}
    public void raiseRings(){ringServo.setPosition(RAISE_ARM2_POS);arm2Up = true;}
    public void initPosition(){
        parkArm();
        closeClaw();
    }
}
