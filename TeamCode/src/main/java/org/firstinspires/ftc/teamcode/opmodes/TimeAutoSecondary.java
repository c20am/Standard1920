package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base_classes.SimpleAutoBot;
import org.firstinspires.ftc.teamcode.enums.Alliance;

/**
 * A time based auto
 **/

public abstract class TimeAutoSecondary extends LinearOpMode {

    public abstract Alliance getAlliance();

    public SimpleAutoBot robot = new SimpleAutoBot(this);
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        //TODO: Set times
        //Power is backwards
        //Turning is backwards
        long AWAY_TIME = 500;
        long TURN_TIME = 400;
        long FORWARD_TIME = 900;
        double SPEED = -.4;
        double TURN_SPEED = -.5;


        robot.initAuto();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // Run until the end of auto (driver presses STOP)
        // changed while to if
        if (opModeIsActive()) {

            //Drive backwards away from wall
            robot.driveForwards(SPEED);
            sleep(AWAY_TIME);
            robot.stopDriving();

            robot.raisePulley(0.5);
            sleep(500);
            robot.stopPulley();
            robot.leftHand.setPosition(.49);
            robot.rightHand.setPosition(.65);

            robot.lowerPulley(0.5);
            sleep(350);
            robot.stopPulley();

            //Rotate
            if (getAlliance() == Alliance.BLUE) {
                //Rotate counterclockwise to point in correct direction
                robot.rotateCounter(TURN_SPEED);
                sleep(TURN_TIME);
                robot.stopDriving();
            } else if (getAlliance() == Alliance.RED) {
                //Rotate clockwise to point in correct direction
                robot.rotateClockwise(TURN_SPEED);
                sleep(TURN_TIME);
                robot.stopDriving();
            }

            robot.driveForwards(SPEED);
            sleep(FORWARD_TIME);
            robot.stopDriving();


        }

    }

}