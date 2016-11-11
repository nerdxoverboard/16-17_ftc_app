package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp: Gage Controller", group="Linear OpMode")
//@Disabled <-- Keep commented unless this Opmode is not in use

public class TeleOpMode extends LinearOpMode {

    /* Declare OpMode Members */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setMaxSpeed(1);
        rightMotor.setMaxSpeed(1);

        // Wait for the game to start (driver plays INIT)
        waitForStart();
        runtime.reset();

        // run until the end of the match
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // If left_stick is pushed up, both motors move forward
            if (1 == 1) { //If this works replace with moveForward();
                leftMotor.setPower(gamepad1.right_stick_x);
                rightMotor.setPower(gamepad1.right_stick_x);

            }
/*
            // If left_stick is pushed down, both motors move in reverse
            else if (gamepad1.left_stick_y == -1.0) { //If this works replace with moveBackwards();
                leftMotor.setPower(gamepad1.left_stick_y);
                rightMotor.setPower(gamepad1.left_stick_y);/*
            }
            else if(gamepad1.right_stick_x == 1.0) { //If this works replace with turnRight();
                leftMotor.setPower(1);
                rightMotor.setPower(-1);
            }

            // If right_stick pushed left,
            // left motor moves in reverse & right motor moves forward
            else if(gamepad1.right_stick_x == -1.0){ //If this works replace with turnLeft();
                leftMotor.setPower(-1);
                rightMotor.setPower(1);*/
            }
        }
}
