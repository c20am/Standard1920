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
        long FORWARD_TIME = 1200;
        //long BACKWARD_TIME = 3000;
        long BACKWARD_TIME= 1500;
        long TURN_TIME_1 = 1500;
        long TURN_TIME_2 = 500;
        long STRAFE_TIME = 1200;
        double SPEED = -.3;
        double TURN_SPEED = .5;
        boolean continuousLowering = true;


        robot.initAuto();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // Run until the end of auto (driver presses STOP)
        // changed while to if
        if (opModeIsActive()) {

            //Start with claws raised
            robot.raiseClaws();

            //Start with pulley raised
            robot.raisePulley(0.5);
            sleep(1000);
            robot.stopPulley();

            if(getAlliance() == Alliance.BLUE) {
                robot.strafeRight(.5);
            } else {
                robot.strafeLeft(.5);
            }
            sleep(500);
            robot.stopDriving();

            //Drive backwards to platform
            robot.driveBackwards(SPEED);
            sleep(FORWARD_TIME);
            robot.stopDriving();

            //Lower claws
            robot.lowerClaws();
            sleep(1200);

            robot.leftHand.setPosition(.49);
            robot.rightHand.setPosition(.65);

            robot.lowerPulley(0.5);
            sleep(1000);
            robot.stopPulley();


            //Drive forwards to building site
            robot.driveForwards(SPEED);
            if (continuousLowering) {
                runtime.reset();
                while (runtime.seconds() < BACKWARD_TIME / 1000) {
                    robot.lowerClaws();
                }
            } else {
                sleep(BACKWARD_TIME);
            }
            robot.stopDriving();


            //Place platform and park

            if (getAlliance() == Alliance.BLUE) {
                //Rotate counterclockwise for .5 seconds to place platform
                robot.rotateCounter(TURN_SPEED);
                if (continuousLowering) {
                    runtime.reset();
                    while (runtime.seconds() < TURN_TIME_1 / 1000) {
                        robot.lowerClaws();
                    }
                } else {
                    sleep(TURN_TIME_1);
                }
                robot.stopDriving();

                //Raise claws
                robot.raiseClaws();
                sleep(1000);

/*
                //Rotate the rest of the wat
                robot.rotateCounter(TURN_SPEED);
                sleep(TURN_TIME_2);
                robot.stopDriving(); */

                //move to park
                //robot.driveForwards(SPEED);
                robot.strafeRight(.6);
                sleep(STRAFE_TIME);
                robot.stopDriving();

            } else if (getAlliance() == Alliance.RED) {
                //Rotate clockwise for .5 seconds to place platform
                robot.rotateClockwise(TURN_SPEED);
                if (continuousLowering) {
                    runtime.reset();
                    while (runtime.seconds() < TURN_TIME_1 / 1000) {
                        robot.lowerClaws();
                    }
                } else {
                    sleep(TURN_TIME_1);
                }
                robot.stopDriving();

                //Raise claws
                robot.raiseClaws();
                sleep(1000);
/*
                //Rotate back
                robot.rotateClockwise(TURN_SPEED);
                sleep(TURN_TIME_2);
                robot.stopDriving();
*/
                //Strafe right to park
                //robot.driveForwards(SPEED);
                robot.strafeLeft(.5);
                sleep(STRAFE_TIME);
                robot.stopDriving();

            }

        }

    }

}