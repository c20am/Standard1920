package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servoTest", group = "Linear Opmode")

public class servoTest extends LinearOpMode {
    Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.servo.get("servo");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_stick_y > .5 && servo.getPosition() < 1) {
                servo.setPosition(servo.getPosition() + .01);
            } else if (gamepad1.left_stick_y < -.5 && servo.getPosition() > -1) {
                servo.setPosition(servo.getPosition() - .01);
            }

            telemetry.addData("servo", servo.getPosition());

            telemetry.update();
        }
    }
}