package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Alliance;

@Autonomous(name = "Auto - Secondary Blue", group = "Auto")
public class SimpleAutoSecondaryBlue extends SimpleAutoSecondary {
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }
}

