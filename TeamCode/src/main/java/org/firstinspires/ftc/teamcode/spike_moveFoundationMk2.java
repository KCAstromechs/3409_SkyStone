package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous (name="spike_moveFoundationMk2")
public class spike_moveFoundationMk2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RobotBaseM1 robotBase = new RobotBaseM1(this);

        waitForStart();

        robotBase.driveStraight(12, 0, 0.675);
        robotBase.driveStraight(10.5, 0, 0.2);
        robotBase.driveStraight(6, 0, 0.2);
        robotBase.stopAndReset();
        robotBase.grabFoundation();
        robotBase.driveStraight(22, 0, -0.95);
        robotBase.driveStraight(24, 0, -0.4);
        robotBase.stopAndReset();
        robotBase.releaseFoundation();
        robotBase.turn(353, 0.8);
        robotBase.turn(358, 0.8);
        robotBase.strafe(60, 0, -0.9);
        /*robotBase.turn(270, 0.7);
        robotBase.driveStraight(8, 270, 0.7);
        robotBase.stopAndReset();
        robotBase.grabFoundation();
        robotBase.driveStraight(20, 270, -0.7);
        robotBase.strafe(6, 270, 0.35);
        robotBase.driveStraight(20, 270, -0.7);*/
    }
}
