package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.base_classes.AutoBot;
import org.firstinspires.ftc.teamcode.enums.Alliance;


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
        telemetry.update();

        robot.init();
        robot.initTracking();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of auto (driver presses STOP)
        while (opModeIsActive()) {
            public void encoderDrive ( double speed, double inches, double timeout){
                int target = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

                if (opModeIsActive()) {
                    robot.frontLeft.setTargetPosition(target);
                    robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                // keep looping while we are still active, and there is time left, and both motors are running.
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

                //  sleep(250);   // optional pause after each move
            }
        }
    }
}
