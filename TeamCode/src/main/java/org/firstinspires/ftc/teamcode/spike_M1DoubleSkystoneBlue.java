package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous (name="spike_M1DoubleSkystoneBlue")
public class spike_M1DoubleSkystoneBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraPos;

        RobotBaseM1 robotBase = new RobotBaseM1(this);

        waitForStart();

        cameraPos = 0;

        robotBase.driveStraight(12, 0, -0.9);
        robotBase.turn(68, 0.6);
        robotBase.stopAndReset();
        switch(cameraPos) {
            case 0:
                robotBase.driveStraight(6, 90, 0.7);
                break;
            case 1:
                robotBase.driveStraight(3, 90, 0.7);
                break;
            case 2:
                break;
        }
        robotBase.turn(158, 0.6);
        robotBase.stopAndReset();
        robotBase.mainFlopDown();
        robotBase.driveStraight(7, 180);
        robotBase.driveStraight(15, 180, 0.2);
        robotBase.stopAndReset();
        robotBase.grabStone();
        robotBase.mainFlopMid();
        robotBase.driveStraight(8, 180, -0.6);
        robotBase.turn(112, 0.6);
        robotBase.stopAndReset();
        switch(cameraPos) {
            case 0:
                robotBase.driveStraight(44, 90);
                robotBase.driveStraight(11, 90, 0.2);
                break;
            case 1:
                robotBase.driveStraight(44, 90);
                robotBase.driveStraight(16, 90, 0.2);
                break;
            case 2:
                break;
        }
        robotBase.turn(158, 0.6);
        robotBase.stopAndReset();
        robotBase.driveStraight(4, 180);
        robotBase.driveStraight(4, 180, 0.15);
        robotBase.stopAndReset();
        robotBase.mainFlopDown();
        robotBase.releaseStone();
        robotBase.driveStraight(8, 180, -0.6);
        robotBase.turn(112, 0.6);
        robotBase.stopAndReset();
        switch(cameraPos) {
            case 0:
                robotBase.driveStraight(65, 90, -0.9);
                robotBase.driveStraight(12, 90, -0.2);
                break;
            case 1:
                robotBase.driveStraight(68, 90, -0.9);
                robotBase.driveStraight(15, 90, -0.2);
                break;
            case 2:
                break;
        }
        robotBase.turn(158, 0.6);
        robotBase.stopAndReset();
        robotBase.driveStraight(4, 180);
        robotBase.driveStraight(8, 180, 0.2);
        robotBase.stopAndReset();
        robotBase.grabStone();
        robotBase.mainFlopMid();
        robotBase.driveStraight(8, 180, -0.6);
        robotBase.turn(112, 0.6);
        robotBase.stopAndReset();
        switch(cameraPos) {
            case 0:
                robotBase.driveStraight(65, 90);
                robotBase.driveStraight(12, 90, 0.2);
                break;
            case 1:
                robotBase.driveStraight(68, 90);
                robotBase.driveStraight(15, 90, 0.2);
                break;
            case 2:
                break;
        }
        robotBase.turn(158, 0.6);
        robotBase.stopAndReset();
        robotBase.lift2F();
        robotBase.driveStraight(6, 180);
        robotBase.driveStraight(4, 180, 0.15);
        robotBase.stopAndReset();
        robotBase.mainFlopDown();
        robotBase.releaseStone();
        robotBase.driveStraight(8, 180, -0.6);
        robotBase.turn(112, 0.6);
        robotBase.stopAndReset();
        robotBase.mainFlopUp();
        robotBase.lift1F();
        robotBase.driveStraight(26, 90, -0.9);
        robotBase.stopAndReset();
    }
}
