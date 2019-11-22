package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base_classes.CoordinateTestTeleBot;
import org.firstinspires.ftc.teamcode.base_classes.TeleBot;

/**
 * This is a test opmode used for getting data from the webcam.
 */
@TeleOp(name = "Basic OpMode", group = "TeleOp")
public class TestTeleOp extends LinearOpMode {

    public CoordinateTestTeleBot robot = new CoordinateTestTeleBot(this);
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.getNav().updateView();

            robot.controlledDrive();
        }
    }
}
