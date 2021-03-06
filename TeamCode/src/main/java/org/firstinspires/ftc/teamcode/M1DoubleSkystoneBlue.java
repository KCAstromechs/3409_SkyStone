package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous (name="M1DoubleSkystoneBlue")
public class M1DoubleSkystoneBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraPos;

        RobotBaseM1 robotBase = new RobotBaseM1(this);
        VisionBaseM1 visionBase = new VisionBaseM1(this);

        waitForStart();

        switch (visionBase.findSkyStone(400, 1280, 350, 420)){
            case LEFT:
                cameraPos = 0;
                break;
            case RIGHT:
                cameraPos = 2;
                break;
            default:                /*---------->>  center and unknown  <<----------*/
                cameraPos = 1;
                break;
        }

        robotBase.driveStraight(4.5, 0, -0.9);
        robotBase.driveStraight(4.5, 0, -0.4);
        robotBase.turn(68, 0.6);
        robotBase.stopAndReset();
        switch(cameraPos) {
            case 0:
                robotBase.driveStraight(2, 90, 0.5);
                robotBase.driveStraight(2, 90, 0.3);
                break;
            case 1:
                robotBase.driveStraight(1, 90, 0.3);
                break;
            case 2:
                robotBase.driveStraight(2, 90, -0.3);
                break;
        }
        robotBase.turn(158, 0.6);
        robotBase.stopAndReset();
        robotBase.pos4();
        robotBase.driveStraight(2, 180, 0.2);
        robotBase.driveStraight(10, 180, 0.2);
        robotBase.stopAndReset();
        robotBase.pos5();
        robotBase.pos3();
        robotBase.driveStraight(4, 180, -0.3);
        robotBase.turn(112, 0.6);
        robotBase.stopAndReset();
        switch(cameraPos) {
            case 0:
                robotBase.driveStraight(28, 90, 0.6);
                robotBase.driveStraight(34.5, 90, 0.2);
                break;
            case 1:
                robotBase.driveStraight(28, 90, 0.6);
                robotBase.driveStraight(46.5, 90, 0.2);
                break;
            case 2:
                robotBase.driveStraight(28, 90, 0.6);
                robotBase.driveStraight(51.5, 90, 0.2);
                break;
        }
        robotBase.turn(158, 0.6);
        robotBase.stopAndReset();
        robotBase.lift1F();
        robotBase.driveStraight(4, 180, 0.3);
        robotBase.stopAndReset();
        robotBase.pos5();
        robotBase.pos4();
        robotBase.driveStraight(4, 180, -0.3);
        robotBase.turn(112, 0.6);
        robotBase.stopAndReset();
        robotBase.liftReset();
        switch(cameraPos) {
            case 0:
                robotBase.driveStraight(28, 90, -0.6);
                robotBase.driveStraight(58, 90, -0.2);
                break;
            case 1:
                robotBase.driveStraight(28, 90, -0.6);
                robotBase.driveStraight(70, 90, -0.2);
                break;
            case 2:
                robotBase.driveStraight(28, 90, -0.6);
                robotBase.driveStraight(35, 90, -0.2); //trying this one
                break;
        }
        robotBase.turn(158, 0.6);
        robotBase.stopAndReset();
        robotBase.driveStraight(8, 180, 0.2);
        robotBase.stopAndReset();
        robotBase.pos5();
        robotBase.pos3();
        robotBase.driveStraight(4, 180, -0.3);
        robotBase.turn(112, 0.6);
        robotBase.stopAndReset();
        switch(cameraPos) {
            case 0:
                robotBase.driveStraight(69, 90);
                robotBase.driveStraight(16, 90, 0.2);
                break;
            case 1:
                robotBase.driveStraight(65, 90);
                robotBase.driveStraight(20, 90, 0.2);
                break;
            case 2:
                robotBase.driveStraight(45, 90);            //trying this one too
                robotBase.driveStraight(12.5, 90, 0.2);
                break;
        }
        robotBase.stopAndReset();
        robotBase.lift2F();
        robotBase.turn(158, 0.6);
        robotBase.stopAndReset();
        robotBase.driveStraight(5, 180, 0.3);
        robotBase.stopAndReset();
        robotBase.pos5();
        robotBase.pos4();
        robotBase.lift3F();
        robotBase.pos1();
        robotBase.liftReset();
        robotBase.driveStraightTime(500, 180, 0.15);
        robotBase.grabFoundation();
        robotBase.driveStraight(11, 180, -0.75);
        robotBase.driveStraight(35, 180, -0.3);
        /*robotBase.driveStraight(4, 180, -0.3);
        robotBase.turn(112, 0.6);
        robotBase.stopAndReset();
        robotBase.pos1();
        robotBase.liftReset();
        robotBase.driveStraight(32, 90, -0.9);
        robotBase.stopAndReset();*/
    }
}
