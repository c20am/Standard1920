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

            //Strafe right to platform
            encoderDrive(.3, 7, 10, Direction.RIGHT);

            //Lower lift
            robot.lowerLift(.8); //TODO: Make lift raise/lower a set amount?
            sleep(1000);
            robot.stopLift();

            //Strafe left to building site
            encoderDrive(.3, 7, 10, Direction.LEFT);

            //Raise lift
            robot.raiseLift(.8);
            sleep(1000);
            robot.stopLift();

        }

    }

    public void encoderDrive(double speed, double inches, double timeout, Direction dir) {
        //TODO: Implement directions
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

        // Stop all motion;
        robot.stopDriving();

        // Turn off RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);   // optional pause after each move
    }

}
