package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base_classes.SimpleAutoBot;
import org.firstinspires.ftc.teamcode.enums.Alliance;

/**
 * A time based auto
 **/

public abstract class TimeAutoTertiary extends LinearOpMode {

    public abstract Alliance getAlliance();

    public SimpleAutoBot robot = new SimpleAutoBot(this);
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        //TODO: Set times
        //Power is backwards
        //Turning is backwards
        long FORWARD_TIME = 1000;
        double SPEED = -.4;


        robot.initAuto();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // Run until the end of auto (driver presses STOP)
        // changed while to if
        if (opModeIsActive()) {

            //Drive under bridge
            robot.driveBackwards(SPEED);
            sleep(FORWARD_TIME);
            robot.stopDriving();


        }

    }

}