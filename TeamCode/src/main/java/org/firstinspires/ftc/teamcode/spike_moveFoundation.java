package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="spike_moveFoundation")
public class spike_moveFoundation extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RobotBaseM1 robotBase = new RobotBaseM1(this);

        waitForStart();

        robotBase.driveStraight(16, 0, 0.675);
        robotBase.driveStraight(1, 0, -0.6);
        robotBase.stopAndReset();
        robotBase.grabFoundation();
        robotBase.arcDrive(9.5, 280, -0.7, false, 1.2);
        robotBase.stopAndReset();
        sleep(1000000);
        ////robotBase.turn(300, 0.7);
        //robotBase.strafe(12, 270, 0.7);
        //robotBase.driveStraight(4, 270);
        //robotBase.stopAndReset();
        ////let go
        //sleep(1100);
        //go to bridge
    }
}
