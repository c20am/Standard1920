package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Alliance;

@Autonomous(name = "Time Auto Secondary - Red", group = "Auto")
public class TimeAutoSecondaryRed extends TimeAutoSecondary {
    public Alliance getAlliance() {
        return Alliance.RED;
    }
}
