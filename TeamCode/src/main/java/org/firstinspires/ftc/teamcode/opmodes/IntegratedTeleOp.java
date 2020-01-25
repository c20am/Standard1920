package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.base_classes.TeleBot;

@TeleOp(name = "intOp", group = "Linear Opmode")
public class IntegratedTeleOp extends LinearOpMode {
    public TeleBot robot = new TeleBot(this);

    controllerPos previousDrive = controllerPos.ZERO;

    static double turnConstant = 1;

    // Define class members
    double strafepower = 1;

    @Override
    public void runOpMode() {
        this.robot.init();
        waitForStart();
        while (opModeIsActive()) {
            moveRobot();
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
            this.robot.stopDriving();
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
        double currentPower = readjustMotorPower(strafedirection * this.strafepower);
        this.robot.strafeLeft(currentPower * strafedirection);
    }

    public void Drive(double drivePower) {
        drivePower = readjustMotorPower(drivePower);
        this.robot.driveForwards(drivePower);
        telemetry.addData("Motors", "drive power (%.2f)", drivePower);
        telemetry.update();
    }

    public void turn(double turn) {
        turn = readjustMotorPower(turn);
        robot.rotateClockwise(turn);
    }

    public void pickup() {
        if (gamepad2.right_stick_y > .5) {
            this.robot.pulley.setPower(.5);

        } else if (gamepad2.right_stick_y < -.5) {
            this.robot.pulley.setPower(-.6);
        } else {
            this.robot.pulley.setPower(0);
        }

        if (gamepad2.a) {
            this.robot.leftHand.setPosition(.16);
            this.robot.rightHand.setPosition(.9);
        }
        if (gamepad2.b) {
            this.robot.leftHand.setPosition(.49);
            this.robot.rightHand.setPosition(.65);
        }

    }

    public void platform() {
        if (gamepad2.left_bumper) {
            this.robot.leftClaw.setPosition(.15);
            this.robot.rightClaw.setPosition(1);
        } else if (gamepad2.right_bumper) {
            this.robot.leftClaw.setPosition(1);
            this.robot.rightClaw.setPosition(.1);
        }

    }

}