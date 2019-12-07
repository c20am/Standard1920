package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.tests.Gyro;

/**
 * driving class - has all basic robot functions
 * created in every base class for runs, ie auto and teleop
 */
public class robot {
    private int encoder = 0;
    private int maxLiftPos = 17250; // add value! equivalent to 6 secs
    private int liftTicks = 0;
    private int pivotTicks = 0;
    private int spoolTicks = 0;

    private static final int minArmPos = -9563; // how far the pivot motor should go
    private static final int maxSpoolPos = 0;
    private static final int middleSpoolPos = 0;

    private static final double out = .2; // servo position to drop team marker
    private static final double in = 1; // servo position to start

    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;
    private DcMotor pulley;
    private DcMotor lift;

    private Servo right;
    private Servo left;

    public Gyro imu;

    /**
     * init - initializes objects in robot,
     *
     * @param hwmap      - hardware map
     * @param initgyro   - whether or not to init gyro
     * @param initvision - whether or not to init vuforia and tFod
     */
    public void init(HardwareMap hwmap, boolean initgyro, boolean initvision) {
        fl = hwmap.get(DcMotor.class, "fl");
        fr = hwmap.get(DcMotor.class, "fr");
        bl = hwmap.get(DcMotor.class, "bl");
        br = hwmap.get(DcMotor.class, "br");

        pulley = hwmap.get(DcMotor.class, "pulley");
        lift = hwmap.get(DcMotor.class, "lift");
        right = hwmap.servo.get("right_hand");
        left = hwmap.servo.get("left_hand");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        if (initgyro) {
            imu = new Gyro(hwmap.get(BNO055IMU.class, "imu"));
        }
    }

    /*
        sets motor powers and readjusts the powers to be in the correct range
     */
    public void drive(double FL, double FR, double BL, double BR) {
        fl.setPower(readjust(FL));
        fr.setPower(readjust(FR));
        bl.setPower(readjust(BL));
        br.setPower(readjust(BR));
    }

    public void drive(double pow) {
        pow = readjust(pow);
        fl.setPower(pow);
        fr.setPower(pow);
        bl.setPower(pow);
        br.setPower(pow);
    }

    /**
     * move robot, depending on given angle and rotation for mecanum chassis
     *
     * @param pow
     * @param angle angle robot should move in from current location, 0 is current forwards position
     * @param rot   rotation robot should move with, 0 is no rotation
     */
    public void move(double pow, double angle, double rot) {
        pow = Range.clip(pow, -1, 1);
        rot = Range.clip(rot, -1, 1);

        double vx = pow * Math.cos(angle + Math.PI / 4); // adding pi/4 ensures that 0 is forwards
        double vy = pow * Math.sin(angle + Math.PI / 4);

        double[] V = {vx + rot, vy - rot, vy + rot, vx - rot}; // vector addition

        double m = 0.0;
        for (double v : V)
            if (Math.abs(v) > m)
                m = v;

        double mult = Math.max(Math.abs(pow), Math.abs(rot));

        if (m != 0)
            for (int i = 0; i < V.length; i++)
                V[i] = Math.abs(mult) * V[i] / Math.abs(m);

        fl.setPower(Range.clip(V[0], -1, 1));
        fr.setPower(Range.clip(V[1], -1, 1));
        bl.setPower(Range.clip(V[2], -1, 1));
        br.setPower(Range.clip(V[3], -1, 1));
    }

    /**
     * moveStraight uses the proportional factor (in PID) in order to correct the angle that the
     * robot moves at
     *
     * @param pow
     * @param angle  the desired angle from forwards that the robot should move in
     * @param actual angle robot is facing (difference from forwards)
     * @param target target angle (generally forwards)
     */
    public void moveStraight(double pow, double angle, double actual, double target) {
        double error = actual - target;

        double P = error * getKp();

        this.move(pow, angle, Range.clip(P, -1, 1));
    }

    // proportionality constant
    private double getKp() {
        return -0.02;
    }

    /**
     * stop drivetrain motors
     */
    public void stop() {
        drive(0, 0, 0, 0);
    }

    // resets encoder value of drivetrain, for moving with ticks
    public void resetTicks() {
        encoder = fl.getCurrentPosition();
    }

    public int getTicks() {
        return fl.getCurrentPosition() - encoder;
    }

    /**
     * maps the input x based on the input and output minimums and maximums, proportionally
     * within the boundaries of the output restraints based on the input restraints
     */
    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
     * returns a value according to the minimum and maximum absolute values of the input, returns
     * based on the sign of the input as well
     */
    public double readjust(double min, double max, double val) {
        if (Math.abs(val) < min && val < 0) {
            return -min;
        } else if (Math.abs(val) < min && val > 0) {
            return min;
        } else if (Math.abs(val) > max && val < 0) {
            return -max;
        } else if (Math.abs(val) > max && val > 0) {
            return max;
        } else {
            return val;
        }
    }

    // default values for motor
    public double readjust(double val) {
        return readjust(.3, 1, val);
    }

}