package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base_classes.AutoBot;
import org.firstinspires.ftc.teamcode.base_classes.SimpleAutoBot;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.Direction;

/**
 * A time based auto
 */

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

            //Strafe right to platform for 3 seconds
            while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                robot.strafeRight(.7);
            }

            //Lower lift
            this.robot.encoderLift(.7, 4000, 10, Direction.DOWN);

            //Strafe left to building site for 3 seconds
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                robot.strafeLeft(.7);
            }

            //Raise lift
            this.robot.encoderLift(.7, 4000, 10, Direction.UP);

            //Park (3 seconds)
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                if (getAlliance() == Alliance.BLUE) {
                    robot.driveBackwards(.7);
                } else if (getAlliance() == Alliance.RED) {
                    robot.driveForwards(.7);
                }
            }

        }

    }

}