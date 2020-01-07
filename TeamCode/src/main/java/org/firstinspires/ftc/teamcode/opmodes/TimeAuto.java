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

        //TODO: Set times
        //Power is backwards
        //Turning is backwards
        long FORWARD_TIME = 500;
        long TURN_TIME = 500;
        long STRAFE_TIME = 500;
        double SPEED = -.4;
        double TURN_SPEED = .5;


        robot.initAuto();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // Run until the end of auto (driver presses STOP)
        while (opModeIsActive()) {

            //Start with claws raised
            robot.raiseClaws();

            //Drive backwards to platform
            robot.driveBackwards(SPEED);
            sleep(FORWARD_TIME);
            robot.stopDriving();

            //Lower claws
            robot.lowerClaws();
            sleep(1000);

            //Drive forwards to building site
            robot.driveForwards(SPEED);
            sleep(FORWARD_TIME);
            robot.stopDriving();


            //Place platform and park
            while (opModeIsActive()) {
                if (getAlliance() == Alliance.BLUE) {
                    //Rotate counterclockwise for .5 seconds to place platform
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < .5)) {
                        robot.rotateCounter(TURN_SPEED);
                        sleep(TURN_TIME);
                        robot.stopDriving();
                    }

                    //Raise claws
                    robot.raiseClaws();
                    sleep(1000);

                    //Rotate back
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < .5)) {
                        robot.rotateClockwise(TURN_SPEED);
                        sleep(TURN_TIME);
                        robot.stopDriving();
                    }

                    //Strafe left to park
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < FORWARD_TIME)) {
                        robot.strafeLeft(SPEED);
                        sleep(STRAFE_TIME);
                        robot.stopDriving();
                    }

                } else if (getAlliance() == Alliance.RED) {
                    //Rotate clockwise for .5 seconds to place platform
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < .5)) {
                        robot.rotateClockwise(TURN_SPEED);
                        sleep(TURN_TIME);
                        robot.stopDriving();
                    }

                    //Raise claws
                    robot.raiseClaws();
                    sleep(1000);

                    //Rotate back
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < .5)) {
                        robot.rotateCounter(TURN_SPEED);
                        sleep(TURN_TIME);
                        robot.stopDriving();
                    }

                    //Strafe right to park
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < FORWARD_TIME)) {
                        robot.strafeRight(SPEED);
                        sleep(STRAFE_TIME);
                        robot.stopDriving();
                    }

                }

            }

        }

    }

}