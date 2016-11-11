/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

// (c) 2016 - FTC Team 11242 - Error 404 - Ferris High School - Ferris, TX

/**
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 *
 * The desired path is:
 * - Shoot 2 particles
 * - Turn at 45 degree angle
 * - Drive straight until we find white line
 * - Straighten up with white line
 * - Get close enough to Beacon
 * - Press correct button with corresponding team color
 * - Turn at 90 degree angle to the right
 * - Drive straight until we find white line
 * - Straighten up with white line
 * - Get close enough to Beacon
 * - Press correct button with corresponding team color
 * - At a [figure out angle] drive backwards to bump ball, if time permits
 */

@Autonomous(name = "Test: PerfectRun", group = "Blue")
//@Disabled <-- Keep commented unless this Opmode is not in use

public class Blue_All_Objectives extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // For NeveRest 40 Gearmotor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                          (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 5.0;      // Full speed
    static final double TURN_SPEED = 5.0;      // Full speed

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        turnAtAngle(45);

        //sleep(2000); // S1: Replicates time for shooting two particles
        //turnByDegrees(45); // S2: Turn at a 45 degree angle
        //encoderDrive(DRIVE_SPEED, 0, -48, 3);  // S3: Drive forward (for now don't worry about color sensor)

        sleep(1000);     // pause for servos to move <-- Is this necessary?

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double inchesToTurn = (45 * (Math.PI / 180) * 18);

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                  (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    public void turnAtAngle(double degrees) throws InterruptedException {

        final double MAX_DEGREES = 360.0;
        final double MIN_DEGREES = 0.0;
        final int    RADIUS = 18;
        int leftTarget;
        int rightTarget;

        double inchesToTurn = (degrees * (Math.PI / 180) * RADIUS);

        leftTarget = robot.leftMotor.getCurrentPosition() + (int) (inchesToTurn * 0);
        rightTarget = robot.rightMotor.getTargetPosition() + (int) (inchesToTurn * COUNTS_PER_INCH);
        robot.leftMotor.setTargetPosition(leftTarget);
        robot.rightMotor.setTargetPosition(rightTarget);

        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //runtime.reset();
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(-1);

        // Turn off RUN_TO_POSITION
        //robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /*public void turnByDegrees(double degrees) throws InterruptedException {
        final double MAX_DEGREES = 360.0; // Highest degree robot can turn
        final double MIN_DEGREES = 0.0;   // Lowest degree robot can turn
        final int RADIUS = 18; // Radius of circle in INCHES, can explain later
        int newLeftTarget;
        int newRightTarget;

        *//*
        Ensure that opmode is still active and number of degrees is between 0 and 360
        *//*
        if(opModeIsActive()) {
            double inchesToTurn = (degrees * (Math.PI / 180) * RADIUS); // Get arc distance in inches
            double leftMotorInches, rightMotorInches;

            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (inchesToTurn * COUNTS_PER_INCH);
            newLeftTarget = robot.leftMotor.getCurrentPosition();
            robot.rightMotor.setTargetPosition(newLeftTarget);
            robot.leftMotor.setTargetPosition(newRightTarget);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            robot.leftMotor.setPower(Math.abs(1.0)); // Max speed
            robot.rightMotor.setPower(Math.abs(1.0)); // Max speed
        }

        // Stop all motion;
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }*/
}
