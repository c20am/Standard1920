package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base_classes.AutoBot;
import org.firstinspires.ftc.teamcode.base_classes.SensorTestBot;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.Direction;


/**
 * An encoder based auto
 */
@Autonomous(name = "Camera Test", group = "Auto")
//@Disabled
public abstract class CamTest extends LinearOpMode {

    public abstract Alliance getAlliance();

    public SensorTestBot robot = new SensorTestBot(this);
    public ElapsedTime runtime = new ElapsedTime();

    //Declares encoder numbers

    @Override
    public void runOpMode() {
/*        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Alliance", getAlliance());
//        telemetry.addData("Test", "test");
//
//        composeTelemetry();
//
//        telemetry.update();*/


        robot.setCam(true);
        robot.initTest();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // Run until the end of auto (driver presses STOP)
        while (opModeIsActive()) {

        }

    }

/*    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
            .addData("roll", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
            .addData("pitch", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }*/

//    /**
//     * Rotates using the IMU in a direction for an angle
//     *
//     * @param speed   the speed (power) to use for the rotation
//     * @param degrees angle in degrees to rotate (between -180 and 180)
//     * @param timeout max time driving before stopping in seconds
//     * @param dir     direction (1 is clockwise, -1 is counterclockwise)
//     */
/*    public void IMURotateForAngle(double speed, double degrees, double timeout, int dir) {

        double target = 0;

        if (opModeIsActive()) {

            // Set target
            target = angles.firstAngle + degrees * dir;

            // Reset the timeout time and start motion.
            runtime.reset();
            robot.rotateClockwise(speed * dir);
        }

        // Keep looping while we are still active, and there is time left, and both motors are running.
        while (opModeIsActive() && (runtime.seconds() < timeout) && dir * (target - angles.firstAngle) > 5) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d", target);
            telemetry.update();
        }

        // Stop all motion
        robot.stopDriving();

        sleep(250);   // optional pause after each move
    }*/

//    /**
//     * Rotates using the IMU in a direction to an angle
//     *
//     * @param speed   the speed (power) to use for the rotation
//     * @param degrees angle in degrees to rotate to (between -180 and 180)
//     * @param timeout max time driving before stopping in seconds
//     */
/*    public void IMURotateToAngle(double speed, double degrees, double timeout) {

        double target = degrees;
        int dir = 0;
        if (Math.abs(target - angles.firstAngle) < 180) {
            dir = (int) Math.signum(target - angles.firstAngle);
        } else {
            dir = -1 * (int) Math.signum(target - angles.firstAngle);
        }
        if (dir != -1 && dir != 1) {
            dir = 1;
        }

        if (opModeIsActive()) {


            // Reset the timeout time and start motion.
            runtime.reset();
            robot.rotateClockwise(speed * dir);
        }

        // Keep looping while we are still active, and there is time left, and both motors are running.
        while (opModeIsActive() && (runtime.seconds() < timeout) && dir * (target - angles.firstAngle) > 5) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d", target);
            telemetry.update();
        }

        // Stop all motion
        robot.stopDriving();

        sleep(250);   // optional pause after each move
    }*/

//    /**
//     * LIFT
//     */
/*    public void encoderLift(double speed, double counts, double timeout, Direction dir) {
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
    }*/
}