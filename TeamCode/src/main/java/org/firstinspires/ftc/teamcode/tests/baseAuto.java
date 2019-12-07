package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tests.robot;

public abstract class baseAuto extends LinearOpMode {
    robot roobot;

    /**
     * initializes robot
     *
     * @throws InterruptedException
     */
    public void setup() throws InterruptedException {
        telemetry.addLine("setup running");
        telemetry.update();
        roobot = new robot();
        roobot.init(hardwareMap, true, true);
        telemetry.addLine("setu up done");
        telemetry.update();
    }

    /**
     * uses encoder to move ticks forwards or backwards relative to initial heading
     *
     * @param pow
     * @param angle   to move in relation to forwards direction
     * @param ticks   number of ticks (related to rotations of the motor) to move
     * @param timeout
     */
    public void moveTicks(double pow, double angle, int ticks, int timeout) {
        roobot.resetTicks();
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        double h = roobot.imu.heading();

        while (ticks > roobot.getTicks() && currentTime - startTime < timeout && opModeIsActive()) {
            roobot.imu.update();
            roobot.moveStraight(pow, angle, roobot.imu.heading(), h);
            currentTime = System.currentTimeMillis();
        }
        roobot.stop();
    }
}