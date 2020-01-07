package org.firstinspires.ftc.teamcode.base_classes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class Robot {
    //TODO: add getters and setters

    LinearOpMode opMode;

    //Declares motors and servos
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor pulley;
    Servo leftHand;
    Servo rightHand;
    Servo leftClaw;
    Servo rightClaw;


    // The IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;

    // Direction of motion
    public double forwardPower = 0;
    public double leftPower = 0;

    public double getForwardPower() {
        return forwardPower;
    }

    public void setForwardPower(double forwardPower) {
        this.forwardPower = forwardPower;
    }

    public double getLeftPower() {
        return leftPower;
    }

    public void setLeftPower(double leftPower) {
        this.leftPower = leftPower;
    }

    public Robot() {

    }

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Initializes telemetry, motors, servos
     */
    public void init() {
        //initializes motors
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "fl");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "fr");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "bl");
        backRight = opMode.hardwareMap.get(DcMotor.class, "br");
        pulley = opMode.hardwareMap.get(DcMotor.class, "pulley");

        //initializes servos
        leftHand = opMode.hardwareMap.get(Servo.class, "left_hand");
        rightHand = opMode.hardwareMap.get(Servo.class, "right_hand");
        leftClaw = opMode.hardwareMap.get(Servo.class, "left_claw");
        rightClaw = opMode.hardwareMap.get(Servo.class, "right_claw");

        //sets direction of the motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        pulley.setDirection(DcMotor.Direction.FORWARD);

        //sets motor encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //FROM ftc samples
        //sets zero power behavior of the motors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void driveForwards(double power) {
        power = Range.clip(power, -1.0, 1.0);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        this.setForwardPower(power);
        this.setLeftPower(0);
    }

    public void driveBackwards(double power) {
        driveForwards(-power);
    }

    public void strafeRight(double power) {
        power = Range.clip(power, -1.0, 1.0);

        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        this.setForwardPower(0);
        this.setLeftPower(power);
    }

    public void strafeLeft(double power) {
        strafeRight(-power);
    }

    public void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        this.setForwardPower(0);
        this.setLeftPower(0);
    }

    public void rotateClockwise(double power) {
        power = Range.clip(power, -1.0, 1.0);

        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        this.setForwardPower(0);
        this.setLeftPower(0);
    }

    public void rotateCounter(double power) {
        rotateClockwise(-power);
    }

    public void lowerClaws() {
        leftClaw.setPosition(.2);
        rightClaw.setPosition(1);

    }

    public void raiseClaws() {
        leftClaw.setPosition(.9);
        rightClaw.setPosition(.3);

    }

    public void raisePulley(double power) {
        power = Range.clip(power, -1.0, 1.0);
        pulley.setPower(power);
    }

    public void lowerPulley(double power) {
        raisePulley(-power);
    }

    public void stopPulley() {
        pulley.setPower(0);
    }

}
