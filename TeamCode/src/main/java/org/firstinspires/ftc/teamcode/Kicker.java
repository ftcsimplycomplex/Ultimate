package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Kicker {
    //Init variables
    private OpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    //Motor Variables
    private Servo kickerArm = null;
    private DcMotor flywheel = null;

    //Servo Positions
    private double restPos = 0.42;
    private double shootPos = 0.64;
    private double flywheelSpeed = 1.0;
    private final double MAX_SPEED = 1.0;
    private final double FLYWHEEL_INCREMENT = 0.05;

// Constructor
    public Kicker(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        kickerArm = hardwareMap.get(Servo.class, "KICKER");
        flywheel = hardwareMap.get(DcMotor.class, "FLYWHEEL");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPower(0.0);
    }

    //rest/init position for kicker
    public void rest() {
        kickerArm.setPosition(restPos);
    }

    //shoot position for kicker
    public void shoot() {
        kickerArm.setPosition(shootPos);
    }

    //Turns on flywheel
    public void flywheel() {
        flywheel.setPower(flywheelSpeed);
    }

    //Turns off flywheel
    public void stopFlywheel() {
        flywheel.setPower(0.0);
    }

    public void increaseFlywheel() {    // bumps the flywheel speed up by one increment
        if (flywheelSpeed <= MAX_SPEED - FLYWHEEL_INCREMENT)
            flywheelSpeed += FLYWHEEL_INCREMENT;
        telemetry.addData("Flywheel speed: ", flywheelSpeed);
        telemetry.update();
    }

    public void decreaseFlywheel() {    // bumps the flywheel speed down by one increment
        if (flywheelSpeed >= FLYWHEEL_INCREMENT)
            flywheelSpeed -= FLYWHEEL_INCREMENT;
        telemetry.addData("Flywheel speed: ", flywheelSpeed);
        telemetry.update();
    }
}
