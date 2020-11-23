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

    private OpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private Servo clawServo;
    private Servo armServo;

    private double OPEN_CLAW_POS = 0.0;
    private double CLOSED_CLAW_POS = 0.0;
    private double PARK_ARM_POS = 0.0;
    private double LIFT_ARM_POS = 0.0;
    private double GRAB_GOAL_POS = 0.0;

    WobbleGoal(OpMode opMode){
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        clawServo = hardwareMap.get(Servo.class, "CLAW");
        armServo = hardwareMap.get(Servo.class, "ARM");
    }
    public void openClaw(){
        clawServo.setPosition(OPEN_CLAW_POS);
    }
    public void closeClaw(){
        clawServo.setPosition(CLOSED_CLAW_POS);

    }
    public void parkArm(){
        clawServo.setPosition(PARK_ARM_POS);

    }
    public void liftArm(){

    }
    public void grabGoal(){

    }
}
