package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.enums.Alliance;

@Disabled
@Autonomous(name = "Auto - Blue", group = "Auto")
public class SimpleAutoBlue extends SimpleAutoMain {
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }
}

