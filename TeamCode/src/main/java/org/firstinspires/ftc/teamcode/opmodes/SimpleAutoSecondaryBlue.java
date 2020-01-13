package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.enums.Alliance;
@Disabled
@Autonomous(name = "Auto - Secondary Blue", group = "Auto")
public class SimpleAutoSecondaryBlue extends SimpleAutoSecondary {
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }
}

