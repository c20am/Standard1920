package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Alliance;

@Autonomous(name = "Time Auto Tertiary - Blue", group = "Auto")
public class TimeAutoTertiaryBlue extends TimeAutoTertiary {
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }
}

