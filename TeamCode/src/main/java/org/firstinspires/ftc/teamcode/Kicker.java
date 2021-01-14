package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


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
    private double flywheelSpeed = 0.45;
    private final double MAX_SPEED = 1.0;
    private final double FLYWHEEL_INCREMENT = 0.05;

// Constructor
    public Kicker(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        kickerArm = hardwareMap.get(Servo.class, "KICKER");
        flywheel = hardwareMap.get(DcMotor.class, "FLYWHEEL");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPower(0.0);
        /*
        * 8-JAN-2021 Joe Levy:
        * Motor direction is affected by selection of motor in Rev configuration screen.  In
        * particular, the two goBILDA motors seem to run in the opposite direction from the others.
        *
        * Further: the encoder direction may be backwards.  I have selected the REVERSE motor direction
        * for the flywheel, and the Matrix 12 Volt motor during configuration.  To match up the
        * encoder direction with the direction the motor turns (required for speed controlled
        * RUN_USING_ENCODER operation) I've intentionally reversed the polarity of the motor power
        * connection.
        *
        * Conceivably, I have an incorrect encoder cable.  There really should be a software setting
        * to allow for this.
        */
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);                                                                // REVERSE for goBilda.
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
//        telemetry.addData("Encoder Ticks: ", flywheel.getCurrentPosition());
//        telemetry.update();
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
