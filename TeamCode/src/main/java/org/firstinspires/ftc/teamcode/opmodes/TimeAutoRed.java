package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Alliance;

@Autonomous(name = "Time Auto - Red", group = "Auto")
public class TimeAutoRed extends TimeAuto {
    public Alliance getAlliance() {
        return Alliance.RED;
    }
}
