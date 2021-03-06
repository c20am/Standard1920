package org.firstinspires.ftc.teamcode.base_classes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "mechh", group = "Linear Opmode")
public class simple extends LinearOpMode {
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor LIFT = null;
    private DcMotor PULLEY = null;
    private Servo left;
    private Servo right;
    private Servo leftClaw;
    private Servo rightClaw;

    controllerPos previousDrive = controllerPos.ZERO;

    static double turnConstant = 1;

    // Define class members
    double strafepower = 1;

    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "fl");
        FR = hardwareMap.get(DcMotor.class, "fr");
        BL = hardwareMap.get(DcMotor.class, "bl");
        BR = hardwareMap.get(DcMotor.class, "br");
        LIFT = hardwareMap.get(DcMotor.class, "lift");
        leftClaw = hardwareMap.get(Servo.class, "right_claw");
        rightClaw = hardwareMap.get(Servo.class, "left_claw");
        PULLEY = hardwareMap.get(DcMotor.class, "pulley");
        right = hardwareMap.servo.get("right_hand");
        left = hardwareMap.servo.get("left_hand");
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PULLEY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        while (opModeIsActive()) {
            moveRobot();
            setLift();
            pickup();
            platform();

            telemetry.update();
            idle();
        }
    }

    public enum controllerPos {
        STRAFE_RIGHT, STRAFE_LEFT, DRIVE_FOWARD, DRIVE_BACK, TURN_RIGHT, TURN_LEFT, ZERO;
    }

    public void moveRobot() {
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x / turnConstant;

        if (drive > 0.2 && (previousDrive == controllerPos.DRIVE_FOWARD || previousDrive == controllerPos.ZERO)) {
            previousDrive = controllerPos.DRIVE_FOWARD;
            Drive(drive);
        } else if (drive < -0.2 && (previousDrive == controllerPos.DRIVE_BACK || previousDrive == controllerPos.ZERO)) {
            previousDrive = controllerPos.DRIVE_BACK;
            Drive(drive);
        } else if (strafe < -.4 && (previousDrive == controllerPos.STRAFE_RIGHT || previousDrive == controllerPos.ZERO)) {
            previousDrive = controllerPos.STRAFE_RIGHT;
            Strafe(-1);
        } else if (strafe > .4 && (previousDrive == controllerPos.STRAFE_LEFT || previousDrive == controllerPos.ZERO)) {
            previousDrive = controllerPos.STRAFE_LEFT;
            Strafe(1);
        } else if (turn > 0.25 && (previousDrive == controllerPos.TURN_RIGHT || previousDrive == controllerPos.ZERO)) {
            previousDrive = controllerPos.TURN_RIGHT;
            turn(turn);
        } else if (turn < -0.25 && (previousDrive == controllerPos.TURN_LEFT || previousDrive == controllerPos.ZERO)) {
            previousDrive = controllerPos.TURN_LEFT;
            turn(turn);
        } else {
            previousDrive = controllerPos.ZERO;
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
        }


    }

    public double readjustMotorPower(double motorPower) {
        motorPower = Range.clip(motorPower, -1.0, 1.0);
        if (Math.abs(motorPower) >= 0.3) {
            return motorPower;
        } else {
            return 0;
        }
    }

    public void Strafe(int strafedirection) {

        double FLpower = readjustMotorPower(-1 * strafedirection * strafepower);
        double FRpower = readjustMotorPower(strafedirection * strafepower);
        double BRpower = readjustMotorPower(-1 * strafedirection * strafepower);
        double BLpower = readjustMotorPower(strafedirection * strafepower);

        FL.setPower(FLpower);
        BL.setPower(BLpower);
        FR.setPower(FRpower);
        BR.setPower(BRpower);

    }

    public void Drive(double drivePower) {
        drivePower = readjustMotorPower(drivePower);
        BL.setPower(drivePower);
        FR.setPower(drivePower);
        FL.setPower(drivePower);
        BR.setPower(drivePower); //can make this an else
        telemetry.addData("Motors", "drive power (%.2f)", drivePower);
        telemetry.update();
    }

    public void turn(double turn) {
        double Rpower = turn;
        double Lpower = -turn;

        Rpower = readjustMotorPower(Rpower);
        Lpower = readjustMotorPower(Lpower);

        FL.setPower(Lpower);
        BL.setPower(Lpower);
        FR.setPower(Rpower);
        BR.setPower(Rpower);
    }

    public void pickup() {
        if (gamepad2.right_stick_y > .5) {
            PULLEY.setPower(.5);
        } else if (gamepad2.right_stick_y < -.5) {
            PULLEY.setPower(-.6);
        } else {
            PULLEY.setPower(0);
        }

        if (gamepad2.a) {
            left.setPosition(.16);
            right.setPosition(.9);
        }
        if (gamepad2.b) {
            left.setPosition(.49);
            right.setPosition(.65);
        }

    }

    public void platform() {
        if (gamepad2.left_bumper) {
            leftClaw.setPosition(.15);
            rightClaw.setPosition(1);
        } else if (gamepad2.right_bumper) {
            leftClaw.setPosition(1);
            rightClaw.setPosition(.1);
        }

    }

    public void setLift() {
        if (gamepad2.right_bumper) {
            LIFT.setPower(1);
        } else if (gamepad2.left_bumper) {
            LIFT.setPower(-1);
        } else {
            LIFT.setPower(0);
        }
    }
}