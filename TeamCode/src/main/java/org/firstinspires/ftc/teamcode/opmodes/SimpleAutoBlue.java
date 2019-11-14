package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Alliance;

@Autonomous(name = "Auto - Blue", group = "Auto")
public class SimpleAutoBlue extends SimpleAutoMain {
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }
}

