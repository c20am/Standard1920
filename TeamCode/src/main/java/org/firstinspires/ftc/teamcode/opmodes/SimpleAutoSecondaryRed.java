package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Alliance;

@Autonomous(name = "Auto - Secondary Red", group = "Auto")
public class SimpleAutoSecondaryRed extends SimpleAutoSecondary {
    public Alliance getAlliance() {
        return Alliance.RED;
    }
}
