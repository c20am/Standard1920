package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.base_classes.AutoBot;
import org.firstinspires.ftc.teamcode.base_classes.SimpleAutoBot;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.Direction;

import java.util.Locale;


/**
 * An encoder based auto
 */

public abstract class SimpleAutoMain extends LinearOpMode {

    public abstract Alliance getAlliance();

    public SimpleAutoBot robot = new SimpleAutoBot(this);
    public ElapsedTime runtime = new ElapsedTime();

    //Declares encoder numbers

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Test", "test");
        telemetry.update();

        robot.initAuto();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        this.robot.getRunTime().reset();


        // Run until the end of auto (driver presses STOP)
        while (opModeIsActive()) {

            // THIS ASSUMES THE LIFT STARTS **UP**

            //TODO: Change how many inches the robot moves (numbers are currently untested and completely arbitrary)
            // and change speed if desired (shouldn't affect distance). Lift movement is tested and is good.

            //Strafe right to platform
            //TODO: Change these inches
            this.robot.encoderDrive(.3, 15, 10, Direction.RIGHT);

            //Lower claws

            //Strafe left to building site
            //TODO: Change these inches
            this.robot.encoderDrive(.3, 15, 10, Direction.LEFT);

            //Raise claws

            //Park
            //TODO: Change these inches
            if (getAlliance() == Alliance.BLUE)
                this.robot.encoderDrive(.7, 15, 10, Direction.BACKWARD);
            else if (getAlliance() == Alliance.RED)
                this.robot.encoderDrive(.7, 15, 10, Direction.FORWARD);


        }

    }


}