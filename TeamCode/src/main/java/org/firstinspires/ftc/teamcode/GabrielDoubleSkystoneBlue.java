package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous (name="GabrielDoubleSkystoneBlue")
public class GabrielDoubleSkystoneBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraPos;

        RobotBaseGabriel robotBase = new RobotBaseGabriel(this);
        VisionBaseGabriel visionBase = new VisionBaseGabriel(this);

        waitForStart();

        /*switch (visionBase.findSkyStone(400, 1280, 350, 420, false)){
            case LEFT:
                cameraPos = 0;
                break;
            case RIGHT:
                cameraPos = 2;
                break;
            default:                *//*---------->>  center and unknown  <<----------*//*
                cameraPos = 1;
                break;
        }*/ cameraPos = 0;
        switch(cameraPos) {
            case 0:
                robotBase.driveStraight(-17, -12, 0, 0.9);
                break;
            case 1:
                robotBase.driveStraight(-17, -4, 0, 0.9);
                break;
            case 2:
                robotBase.driveStraight(-17, 4, 0, 0.9);
                break;
        }
        robotBase.turn(158, 0.6);
        robotBase.stopAndReset();
        robotBase.pos4();
        robotBase.driveStraight(robotBase.getDistSensorInch(), 180, 0.3);
        robotBase.stopAndReset();
        robotBase.pos5();
        robotBase.pos3();
        robotBase.driveStraight(6, 180, -0.6);
        robotBase.turn(112, 0.6);
        robotBase.stopAndReset();
        switch(cameraPos) {
            case 0:
                robotBase.driveStraight(77, 90, 0.9);
                break;
            case 1:
                robotBase.driveStraight(85, 90, 0.9);
                break;
            case 2:
                robotBase.driveStraight(93, 90, 0.9);
                break;
        }
        robotBase.turn(158, 0.6);
        robotBase.stopAndReset();
        robotBase.lift1F();
        robotBase.driveStraight(robotBase.getDistSensorInch(), 180, 0.3);
        robotBase.stopAndReset();
        robotBase.pos5();
        robotBase.pos4();
        robotBase.driveStraight(6, 180, -0.6);
        robotBase.turn(112, 0.6);
        robotBase.stopAndReset();
        robotBase.liftReset();
        switch(cameraPos) {
            case 0:
                robotBase.driveStraight(100, 90, -0.9);
                break;
            case 1:
                robotBase.driveStraight(108, 90, -0.9);
                break;
            case 2:
                robotBase.driveStraight(76, 90, -0.9);
                break;
        }
        robotBase.turn(158, 0.6);
        robotBase.stopAndReset();
        robotBase.driveStraight(robotBase.getDistSensorInch(), 180, 0.3);
        robotBase.stopAndReset();
        robotBase.pos5();
        robotBase.pos3();
        robotBase.driveStraight(6, 180, -0.6);
        robotBase.turn(112, 0.6);
        robotBase.stopAndReset();
        switch(cameraPos) {
            case 0:
                robotBase.driveStraight(103, 90);
                break;
            case 1:
                robotBase.driveStraight(111, 90);
                break;
            case 2:
                robotBase.driveStraight(79, 90);
                break;
        }
        robotBase.stopAndReset();
        robotBase.lift2F();
        robotBase.turn(158, 0.6);
        robotBase.stopAndReset();
        robotBase.driveStraight(robotBase.getDistSensorInch(), 180, 0.3);
        robotBase.stopAndReset();
        robotBase.pos5();
        robotBase.pos4();
        robotBase.lift3F();
        robotBase.pos1();
        robotBase.liftReset();
        robotBase.driveStraightTime(500, 180, 0.15);
        robotBase.grabFoundation();
        robotBase.driveStraight(30, 180, -0.9);
        /*robotBase.driveStraight(4, 180, -0.4);
        robotBase.turn(112, 0.6);
        robotBase.stopAndReset();
        robotBase.pos1();
        robotBase.liftReset();
        robotBase.driveStraight(32, 90, -0.9);
        robotBase.stopAndReset();*/
    }
}
