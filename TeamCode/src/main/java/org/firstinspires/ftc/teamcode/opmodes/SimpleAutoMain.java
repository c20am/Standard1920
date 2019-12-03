package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.base_classes.AutoBot;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.Direction;


/**
 * An encoder based auto
 */

public abstract class SimpleAutoMain extends LinearOpMode {

    public abstract Alliance getAlliance();

    public AutoBot robot = new AutoBot(this);
    public ElapsedTime runtime = new ElapsedTime();

    //Declares encoder numbers
    final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_INCHES = 3.94;     // For figuring circumference
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    final double LIFT_COUNTS_PER_INCH = 5000;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Test", "test");

        telemetry.update();

        robot.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of auto (driver presses STOP)
        if (opModeIsActive()) {

            // THIS ASSUMES THE LIFT STARTS **UP**

            //TODO: Change how many inches the robot moves (numbers are currently untested and completely arbitrary)
            // and change speed if desired (shouldn't affect distance). Lift movement is tested and is good.

            //Strafe right to platform
            //TODO: Change these inches
            encoderDrive(.3, 15, 10, Direction.RIGHT);

            //Lower lift
            encoderLift(.7, 5000, 10, Direction.DOWN);

            //Strafe left to building site
            //TODO: Change these inches
            encoderDrive(.3, 15, 10, Direction.LEFT);

            //Raise lift
            encoderLift(.7, 5000, 10, Direction.UP);

            //Park
            //TODO: Change these inches
            if (getAlliance() == Alliance.BLUE) encoderDrive(.7, 15, 10, Direction.BACKWARD);
            else if (getAlliance() == Alliance.RED) encoderDrive(.7, 15, 10, Direction.FORWARD);

        }

    }

    public void encoderDrive(double speed, double inches, double timeout, Direction dir) {

        int target = 0;

        if (opModeIsActive()) {

            // Set target
            target = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            // Turn On RUN_TO_POSITION
            if (dir == Direction.BACKWARD) target = -target;
            else if (dir == Direction.RIGHT) target = -target;
            robot.frontLeft.setTargetPosition(target);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion.
            runtime.reset();
            if (dir == Direction.FORWARD) robot.driveForwards(speed);
            else if (dir == Direction.BACKWARD) robot.driveBackwards(speed);
            else if (dir == Direction.RIGHT) robot.strafeRight(speed);
            else if (dir == Direction.LEFT) robot.strafeLeft(speed);

        }

        // Keep looping while we are still active, and there is time left, and both motors are running.
        while (opModeIsActive() && (runtime.seconds() < timeout) && (robot.frontLeft.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d", target);
            telemetry.addData("Path2", "Running at %7d", robot.frontLeft.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion
        robot.stopDriving();

        // Turn off RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);   // optional pause after each move
    }

    /**
     * LIFT
     */
    public void encoderLift(double speed, double counts, double timeout, Direction dir) {
        int target = 0;

        if (opModeIsActive()) {

            // Set target
            target = robot.lift.getCurrentPosition() + (int) counts;
            if (dir == Direction.DOWN) target = -target;

            // Turn On RUN_TO_POSITION
            robot.lift.setTargetPosition(target);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion.
            runtime.reset();
            if (dir == Direction.UP) robot.raiseLift(speed);
            else if (dir == Direction.DOWN) robot.lowerLift(speed);

        }

        // Keep looping while we are still active, and there is time left, and both motors are running.
        while (opModeIsActive() && (runtime.seconds() < timeout) && (robot.lift.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Lift Running to %7d", target);
            telemetry.addData("Path2", "Lift Running at %7d", robot.lift.getCurrentPosition());
            telemetry.update();
        }

        // Stop lift
        robot.stopLift();

        // Turn off RUN_TO_POSITION
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);   // optional pause after each move

    }
}