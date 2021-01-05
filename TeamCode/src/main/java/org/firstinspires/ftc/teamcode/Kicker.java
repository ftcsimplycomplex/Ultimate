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

public class Kicker {
    //Init varibles
    private OpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    //Motor Varibles
    private Servo Kicker= null;
    private DcMotor flywheel;
    //Servo Positions
    private double restPos= 0.42;
    private double shootPos= 0.64;


    public Kicker (OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        Kicker= hardwareMap.get(Servo.class, "KICKER");
    }
    //rest/init position for kicker
    public void rest () {
        Kicker.setPosition(restPos);
    }
    //shoot position for kicker
    public void shoot () {
        Kicker.setPosition(shootPos);
    }
    //Turns on flywheel
    public void flywheel() {
//        flywheel.setPower(1);
    }
    //Turns off flywheel
    public void stopFlywheel () {
//        flywheel.setPower(0);
    }
}
