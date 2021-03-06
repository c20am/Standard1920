package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.enums.Alliance;
@Disabled
@Autonomous(name = "Auto - Red", group = "Auto")
public class SimpleAutoRed extends SimpleAutoMain {
    public Alliance getAlliance() {
        return Alliance.RED;
    }
}
