package org.firstinspires.ftc.teamcode.base_classes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.algorithms.IMU;
import org.firstinspires.ftc.teamcode.algorithms.VuforiaAutoNav;
import org.firstinspires.ftc.teamcode.enums.Direction;

import static org.firstinspires.ftc.teamcode.enums.Direction.BACKWARD;
import static org.firstinspires.ftc.teamcode.enums.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.enums.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.enums.Direction.RIGHT;

/**
 * This class is the class for simple autonomous robots
 * <p>
 * TODO: implement bound checks for robot (based on math for size of robot)
 */
public class SimpleAutoBot extends Robot {

    //stores the value of sin(45°), or sin(pi/4)
    public double sin45 = Math.sqrt(2) / 2;
    private float robotWidth = 15.68f; // robot width in inches
    private float robotLength = 16.25f; // robot length in inches
    //Declares encoder numbers
    final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_INCHES = 3.94;     // For figuring circumference
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    final double LIFT_COUNTS_PER_INCH = 5000;
    private int alliance = 1;

    private ElapsedTime runtime = new ElapsedTime();

    /**
     * gets the width
     *
     * @return width
     */
    public float getRobotWidth() {
        return robotWidth;
    }

    /**
     * sets the width
     *
     * @param robotWidth
     */
    public void setRobotWidth(float robotWidth) {
        this.robotWidth = robotWidth;
    }

    /**
     * gets the length
     *
     * @return length
     */
    public float getRobotLength() {
        return robotLength;
    }

    /**
     * sets the length
     *
     * @param robotLength
     */
    public void setRobotLength(float robotLength) {
        this.robotLength = robotLength;
    }

    /**
     * gets the angled width
     * TODO: make sure this works with new coordinate system
     *
     * @param angle angle, in degrees, that the robot is rotated counterclockwise from its original position
     * @return angled width
     */
    public float getRobotAngledWidth(double angle) {
        return (float) Math.abs(this.getRobotWidth() * Math.cos(Math.toRadians(-1 * (double) angle - 90))) + (float) Math.abs(this.getRobotLength() * Math.sin(Math.toRadians(-1 * (double) angle - 90)));
    }

    /**
     * gets the angled length
     * TODO: make sure this works with new coordinate system
     *
     * @param angle angle, in degrees, that the robot is rotated counterclockwise from its original position
     * @return angled length
     */
    public float getRobotAngledLength(double angle) {
        return (float) Math.abs(this.getRobotWidth() * Math.sin(Math.toRadians(-1 * (double) angle - 90))) + (float) Math.abs(this.getRobotLength() * Math.cos(Math.toRadians(-1 * (double) angle - 90)));
    }


    /**
     * constructor for auto bot
     *
     * @param opMode
     */
    public SimpleAutoBot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * gets the motor powers
     *
     * @return motor powers
     */
    public double[] getPowers() {
        return new double[]{frontLeft.getPower(), frontRight.getPower(), backRight.getPower(), backLeft.getPower()};
    }

    /**
     * gets the run time
     *
     * @return run time
     */
    public ElapsedTime getRunTime() {
        return runtime;
    }

    /**
     * sets the run time
     *
     * @param runtime
     */
    public void setRunTime(ElapsedTime runtime) {
        this.runtime = runtime;
    }

    /**
     * Initializes the robot (Use this instead of .init())
     */
    public void initAuto() {
        this.init();
    }


    /**
     * LIFT
     */
    public void encoderLift(double speed, double counts, double timeout, Direction dir) {
        int target = 0;

        if (this.opMode.opModeIsActive()) {

            // Set target
            target = this.lift.getCurrentPosition() + (int) counts;
            if (dir == Direction.DOWN) target = -target;

            // Turn On RUN_TO_POSITION
            this.lift.setTargetPosition(target);
            this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion.
            this.getRunTime().reset();
            if (dir == Direction.UP) this.raiseLift(speed);
            else if (dir == Direction.DOWN) this.lowerLift(speed);

        }

        // Keep looping while we are still active, and there is time left, and both motors are running.
        while (this.opMode.opModeIsActive() && (this.getRunTime().seconds() < timeout) && (this.lift.isBusy())) {

            // Display it for the driver.
            this.opMode.telemetry.addData("Path1", "Lift Running to %7d", target);
            this.opMode.telemetry.addData("Path2", "Lift Running at %7d", this.lift.getCurrentPosition());
            this.opMode.telemetry.update();
        }

        // Stop lift
        this.stopLift();

        // Turn off RUN_TO_POSITION
        this.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.opMode.sleep(250);   // optional pause after each move

    }

    /**
     * TODO: make sure that this also works for strafing
     *
     * @param speed
     * @param inches
     * @param timeout
     * @param dir
     */
    public void encoderDrive(double speed, double inches, double timeout, Direction dir) {

        int target = 0;
        int current = 0;
        if (this.opMode.opModeIsActive()) {
            // Set target
            target = this.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            current = this.frontLeft.getCurrentPosition();

            // Turn On RUN_TO_POSITION
            if (dir == Direction.BACKWARD) target = -target;
            else if (dir == Direction.RIGHT) target = -target;
            this.frontLeft.setTargetPosition(target);
            this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion.
            this.getRunTime().reset();
            if (dir == FORWARD) this.driveForwards(speed);
            else if (dir == Direction.BACKWARD) this.driveBackwards(speed);
            else if (dir == Direction.RIGHT) this.strafeRight(speed);
            else if (dir == LEFT) this.strafeLeft(speed);

        }

        // Keep looping while we are still active, and there is time left, and both motors are running.
        while (this.opMode.opModeIsActive() && (this.getRunTime().seconds() < timeout) && (this.frontLeft.isBusy())) {

            // Display it for the driver.
            this.opMode.telemetry.addData("Path1", "Running to %7d", target);
            this.opMode.telemetry.addData("Path2", "Running at %7d", this.frontLeft.getCurrentPosition());
            this.opMode.telemetry.update();
        }

        // Stop all motion
        this.stopDriving();

        // Turn off RUN_TO_POSITION
        this.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.opMode.sleep(250);   // optional pause after each move
    }

}
