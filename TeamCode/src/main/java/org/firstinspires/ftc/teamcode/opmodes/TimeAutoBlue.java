package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Alliance;

@Autonomous(name = "Time Auto - Blue", group = "Auto")
public class TimeAutoBlue extends TimeAuto {
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }
}

