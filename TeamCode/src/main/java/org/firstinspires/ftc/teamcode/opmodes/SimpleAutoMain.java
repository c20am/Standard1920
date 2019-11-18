package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.base_classes.AutoBot;
import org.firstinspires.ftc.teamcode.enums.Alliance;


/**
 * A very simple time-based backup auto
 *
 * Untested, so timing is arbitrary
 */

public abstract class SimpleAutoMain extends LinearOpMode {

    public abstract Alliance getAlliance();

    public AutoBot robot = new AutoBot(this);
    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", getAlliance());
        telemetry.update();

        robot.init();
        robot.initTracking();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of auto (driver presses STOP)
        while (opModeIsActive() && (runtime.seconds() < 1.0)) { //Raise lift for 1 second
            robot.raiseLift(.5);
        }

        while (opModeIsActive() && (runtime.seconds() < 3.0)) { //Strafe for 2 seconds to reach platform
            robot.stopLift();
            if (getAlliance() == Alliance.RED) robot.strafeLeft(.75);
            if (getAlliance() == Alliance.BLUE) robot.strafeRight(.75);
        }

        while (opModeIsActive() && (runtime.seconds() < 4.0)) { //Lower lift for 1 second to grab platform
            robot.stopDriving();
            robot.lowerLift(.75);
        }

        while (opModeIsActive() && (runtime.seconds() < 6.0)) { //Strafe for 2 seconds to place platform
            robot.stopLift();
            if (getAlliance() == Alliance.RED) robot.strafeRight(.75);
            if (getAlliance() == Alliance.BLUE) robot.strafeLeft(.75);
        }
    }
}
