package org.firstinspires.ftc.teamcode.base_classes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.algorithms.IMU;
import org.firstinspires.ftc.teamcode.algorithms.VuforiaAutoNav;
import org.firstinspires.ftc.teamcode.enums.Direction;

import static org.firstinspires.ftc.teamcode.enums.Direction.BACKWARD;
import static org.firstinspires.ftc.teamcode.enums.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.enums.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.enums.Direction.RIGHT;

/**
 * This class is the class for complex autonomous robots
 * <p>
 * TODO: implement bound checks for robot (based on math for size of robot)
 */
public class AutoBot extends Robot {

    //stores the value of sin(45°), or sin(pi/4)
    public double sin45 = Math.sqrt(2) / 2;
    private VuforiaAutoNav nav;
    private IMU imu;
    private float robotWidth = 15.68f; // robot width in inches
    private float robotLength = 16.25f; // robot length in inches
    //Declares encoder numbers
    final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_INCHES = 3.94;     // For figuring circumference
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    final double LIFT_COUNTS_PER_INCH = 5000;
    private int alliance = 1;

    private float x = 0;
    private float y = 0;
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * gets the x
     *
     * @return x
     */
    public float getX() {
        return x;
    }

    /**
     * sets the x
     *
     * @param x
     */
    public void setX(float x) {
        this.x = x;
    }

    /**
     * gets the y
     *
     * @return y
     */
    public float getY() {
        return y;
    }

    /**
     * sets the y
     *
     * @param y
     */
    public void setY(float y) {
        this.y = y;
    }


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
    public AutoBot(LinearOpMode opMode) {
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
     * gets the auto nav
     *
     * @return auto nav
     */
    public VuforiaAutoNav getNav() {
        return nav;
    }

    /**
     * sets the auto nav
     *
     * @param nav
     */
    public void setNav(VuforiaAutoNav nav) {
        this.nav = nav;
    }

    /**
     * gets the imu
     *
     * @return imu
     */
    public IMU getIMU() {
        return imu;
    }

    /**
     * sets the imu
     *
     * @param imu
     */
    public void setIMU(IMU imu) {
        this.imu = imu;
    }

    /**
     * gets the nav angle
     *
     * @return angle
     */
    public float getNavAngle() {
        return this.getNav().getRobotAngle();
    }

    /**
     * sets the nav angle
     *
     * @param angle`
     */
    public void setNavAngle(float angle) {
        this.getNav().setRobotAngle(angle);
    }

    /**
     * gets the imu angle
     *
     * @return angle
     */
    public float getIMUAngle() {
        return this.getIMU().getAngle();
    }

    /**
     * sets the imu angle
     *
     * @param angle
     */
    public void setIMUAngle(float angle) {
        this.getIMU().setAngle(angle);
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
     * changes the coordinates as if the robot was moved for a distance in a direction (based on current angle)
     *
     * @param distance distance to move
     * @param dir      direction to move in
     */
    public void moveRobot(float distance, Direction dir) {
        if (dir == FORWARD) {
            this.setX(this.getX() + (float) (distance * Math.sin((double) this.getIMUAngle())));
            this.setY(this.getY() + (float) (distance * Math.cos((double) this.getIMUAngle())));
        } else if (dir == LEFT) {
            this.setX(this.getX() + (float) (distance * Math.sin((double) this.getIMUAngle() + 90)));
            this.setY(this.getY() + (float) (distance * Math.cos((double) this.getIMUAngle() + 90)));
        } else if (dir == BACKWARD) {
            this.setX(this.getX() + (float) (distance * Math.sin((double) this.getIMUAngle() + 180)));
            this.setY(this.getY() + (float) (distance * Math.cos((double) this.getIMUAngle() + 180)));
        } else if (dir == RIGHT) {
            this.setX(this.getX() + (float) (distance * Math.sin((double) this.getIMUAngle() + 270)));
            this.setY(this.getY() + (float) (distance * Math.cos((double) this.getIMUAngle() + 270)));
        }

    }

    /**
     * Initializes the robot (Use this instead of .init())
     */
    public void initAuto() {
        this.init();
        this.setNav(new VuforiaAutoNav(this.opMode.hardwareMap, this.opMode.telemetry));
        this.setIMU(new IMU(this.opMode.hardwareMap, this.opMode.telemetry));
        this.getNav().initView();
        this.getIMU().initIMU();
        this.getNav().updateView();
        this.setX(this.getNav().getRobotX());
        this.setY(this.getNav().getRobotY());
        this.getIMU().setAngle(this.getNavAngle());
    }

    /**
     * Rotates using the IMU in a direction for an angle
     *
     * @param speed   the speed (power) to use for the rotation
     * @param degrees angle in degrees to rotate (between 0 and 359)
     * @param timeout max time driving before stopping in seconds
     * @param dir     direction (1 is clockwise, -1 is counterclockwise)
     */
    public void IMURotateForAngle(double speed, double degrees, double timeout, int dir) {

        double target = 0;

        if (this.opMode.opModeIsActive()) {

            // Set target
            target = this.getIMU().getAngles().firstAngle + degrees * dir;

            // Reset the timeout time and start motion.
            this.getRunTime().reset();
            this.rotateCounter(speed * dir);
        }

        // Keep looping while we are still active, and there is time left, and both motors are running.
        while (this.opMode.opModeIsActive() && (this.getRunTime().seconds() < timeout) && dir * (target - this.getIMU().getAngles().firstAngle) > 5) {

            // Display it for the driver.
            this.opMode.telemetry.addData("Path1", "Running to %7d", target);
            this.opMode.telemetry.update();
        }

        // Stop all motion
        this.stopDriving();

        this.opMode.sleep(250);   // optional pause after each move
    }

    /**
     * Rotates using the IMU in a direction to an angle
     *
     * @param speed   the speed (power) to use for the rotation
     * @param degrees angle in degrees to rotate to (between 0 and 359)
     * @param timeout max time driving before stopping in seconds
     */
    public void IMURotateToAngle(double speed, double degrees, double timeout) {

        double target = degrees;
        int dir = 0;
        if (Math.abs(target - this.getIMU().getAngles().firstAngle) < 180) {
            dir = (int) Math.signum(target - this.getIMU().getAngles().firstAngle);
        } else {
            dir = -1 * (int) Math.signum(target - this.getIMU().getAngles().firstAngle);
            dir = -1 * (int) Math.signum(target - this.getIMU().getAngles().firstAngle);
        }
        if (dir != -1 && dir != 1) {
            dir = 1;
        }

        if (this.opMode.opModeIsActive()) {


            // Reset the timeout time and start motion.
            this.getRunTime().reset();
            this.rotateCounter(speed * dir);
        }

        // Keep looping while we are still active, and there is time left, and both motors are running.
        while (this.opMode.opModeIsActive() && (this.getRunTime().seconds() < timeout) && dir * (target - this.getIMU().getAngles().firstAngle) > 5) {

            // Display it for the driver.
            this.opMode.telemetry.addData("Path1", "Running to %7d", target);
            this.opMode.telemetry.update();
        }

        // Stop all motion
        this.stopDriving();

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
        this.moveRobot((float) (Math.abs(this.frontLeft.getCurrentPosition() - current) / COUNTS_PER_INCH), dir);

        // Turn off RUN_TO_POSITION
        this.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.opMode.sleep(250);   // optional pause after each move
    }

}
