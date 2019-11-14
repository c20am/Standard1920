package org.firstinspires.ftc.teamcode.base_classes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

public class TeleBot extends Robot {

    //under this number, input won't be taken into account
    public double dead = .13;

    //stores the value of sin(45°), or sin(pi/4)
    public double sin45 = Math.sqrt(2) / 2;

    public TeleBot(OpMode opMode) {
        this.opMode = opMode;
    }

    public double[] getPowers() {
        return new double[]{frontLeft.getPower(), frontRight.getPower(), backRight.getPower(), backLeft.getPower()};
    }

    /**
     * Takes controller input in order to move the robot
     */
    public void controlledDrive() {
        //stores values once so you don't have to retrieve them again within the method
        double leftX = opMode.gamepad1.left_stick_x;
        double leftY = opMode.gamepad1.left_stick_y;
        double rightX = opMode.gamepad1.right_stick_x;
        boolean dpadUp = opMode.gamepad1.dpad_up;
        boolean dpadDown = opMode.gamepad1.dpad_down;
        boolean leftBumper = opMode.gamepad1.left_bumper;
        boolean rightBumper = opMode.gamepad1.right_bumper;

        //the 'radius' of the circle this represents
        double rad = 1;

        //power for the left hand mecanum wheels (front left, back right)
        double strafeLeftPow = 0;

        //power for the right hand mecanum wheels (front right, back left)
        double strafeRightPow = 0;

        //power difference dedicated to rotation
        double rotPow = 0;

        //stores the power given to each motor clockwise from front left
        double[] powerValues;

        if ((Math.abs(leftX) > dead) || (Math.abs(leftY) > dead) || Math.abs(rightX) > dead) {

            //Move robot according to left stick
            if (Math.abs(leftX) < leftY) {
                driveForwards(.75);
            } else if (Math.abs(leftY) < leftX) {
                strafeRight(.75);
            } else if (-Math.abs(leftY) > leftX) {
                strafeLeft(.75);
            } else if (-Math.abs(leftX) > leftY) {
                driveBackwards(.75);
            } else {
                stopDriving();
            }

            //Powers pulley's motor according to dpad
            if (dpadUp) {
                raisePulley(.5);
            } else if (dpadDown) {
                lowerPulley(.5);
            } else {
                stopPulley();
            }

            //Powers lift's motor according to shoulder buttons
            if (rightBumper) {
                raiseLift(.8);
            } else if (leftBumper) {
                lowerLift(.8);
            } else {
                stopLift();
            }

            //TODO: Add servos/hands movement when we figure out where 0 degrees is on the servos

        }
    }
}