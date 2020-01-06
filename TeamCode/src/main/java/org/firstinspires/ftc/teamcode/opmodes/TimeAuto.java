package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base_classes.SimpleAutoBot;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.Direction;

/**
 * A time based auto
 **/

public abstract class TimeAuto extends LinearOpMode {

    public abstract Alliance getAlliance();

    public SimpleAutoBot robot = new SimpleAutoBot(this);
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.initAuto();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // Run until the end of auto (driver presses STOP)
        while (opModeIsActive()) {

            /** THIS ASSUMES THE LIFT STARTS UP **/
            //TODO: Set times

            //Drive backwards to platform for 3 seconds
            while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                robot.driveBackwards(.7);
            }

            //Lower lift
            this.robot.encoderLift(.7, 4000, 10, Direction.DOWN);

            //Drive forwards to building site for 3 seconds
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                robot.driveForwards(.7);
            }

            //Raise lift
            this.robot.encoderLift(.7, 4000, 10, Direction.UP);

            //Place platform and park
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                if (getAlliance() == Alliance.BLUE) {
                    //Rotate counterclockwise for .5 seconds to place platform
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < .5)) {
                        robot.rotateCounter(.7);
                    }

                    //Rotate back
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < .5)) {
                        robot.rotateClockwise(.7);
                    }

                    //Strafe left for 3 seconds to park
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < 3)) {
                        robot.strafeLeft(.7);
                    }

                } else if (getAlliance() == Alliance.RED) {
                    //Rotate clockwise for .5 seconds to place platform
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < .5)) {
                        robot.rotateClockwise(.7);
                    }

                    //Rotate back
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < .5)) {
                        robot.rotateCounter(.7);
                    }

                    //Strafe right for 3 seconds to park
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < 3)) {
                        robot.strafeRight(.7);
                    }

                }
            }

            //Stop
            robot.stopDriving();

        }

    }

}