package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="CygnusBlue")
public class CygnusBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraPos;

        RobotBaseCygnus rb = new RobotBaseCygnus(this);
        VisionBaseCygnus vb = new VisionBaseCygnus(this);
        telemetry.addData("ready", true);
        telemetry.update();

        waitForStart();

        switch (vb.findSkyStone(430, 1170, 330, 470, false)){
            case LEFT:
                rb.driveTo(-13.5, 18, 323, 1);
                rb.grabOpen();
                rb.stop();
                rb.driveTo(-13.5, 29.75, 0, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(-13.5, 25, 0, 1);
                rb.driveTo(-77, 25, 90, 1);
                rb.stop();
                rb.setLiftTarget(1, 50);
                rb.driveTo(-77, 25,0, 1);
                rb.driveTo(-77, 28, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(-77, 25, 0, 1);
                rb.setLiftTarget(0, 45);
                rb.driveTo(10.5, 25, 90, 1);
                rb.driveTo(10.5, 25, 0, 1);
                rb.stop();
                rb.driveTo(10.5, 33, 0, 1);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(10.5, 25, 0, 1);
                rb.driveTo(-89, 25, 90, 1);
                rb.stop();
                rb.setLiftTarget(1, 50);
                rb.driveTo(-89, 25, 0, 1);
                rb.driveTo(-89, 31, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                /*rb.setLiftTarget(0, 45);
                rb.driveTo(-89, 25, 0, 1);
                rb.driveTo(-39, 25, 270, 1);
                rb.stop();
                rb.setLiftTarget(0, 0);
                double startTime = getRuntime();
                while(getRuntime()<startTime+0.7 && opModeIsActive()){
                    rb.lifterHandler();
                }*/
                rb.setLiftTarget(0, 30);
                double startTime = getRuntime();
                rb.driveTo(-89, 33, 0, 1);
                rb.stop();
                while(getRuntime()<startTime+0.7){
                    rb.lifterHandler();
                }
                rb.driveTo(-77, 19.5, 311, 1);
                rb.turn(270, 1);
                rb.driveTo(-80, 19.5, 270, 1);
                rb.stop();
                rb.setLiftTarget(1, 0);
                while(getRuntime()<startTime+0.7){
                    rb.lifterHandler();
                }
                rb.setLiftTarget(0, 0);
                rb.driveTo(-39, 28, 270, 1);
                rb.stop();
                break;
            case RIGHT:
                rb.driveTo(2.5, 18, 8, 1);
                rb.grabOpen();
                rb.stop();
                rb.driveTo(2.5, 29.75, 0, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(2.5, 25, 0, 1);
                rb.driveTo(-77, 25, 90, 1);
                rb.stop();
                rb.setLiftTarget(1, 100);
                rb.driveTo(-77, 25,0, 1);
                rb.driveTo(-77, 28, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(-77, 23, 0, 1);
                rb.setLiftTarget(0, 45);
                rb.driveTo(/*18.5*/17.1, 25, 270, 1);
                rb.driveTo(17.1, 25, 30, 1);
                rb.stop();
                /*rb.driveTo(18.5, 28, 0, 1);*/
                rb.driveTo(20, 34, 30, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(18.5, 25, 0, 1);
                rb.driveTo(-89, 25, 90, 1);
                rb.stop();
                rb.setLiftTarget(1, 50);
                rb.driveTo(-89, 25, 0, 1);
                rb.driveTo(-89, 31, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                /*rb.setLiftTarget(0, 45);
                rb.driveTo(-89, 25, 0, 1);
                rb.driveTo(-39, 25, 270, 1);
                rb.stop();
                rb.setLiftTarget(0, 0);
                startTime = getRuntime();
                while(getRuntime()<startTime+0.7 && opModeIsActive()){
                    rb.lifterHandler();
                }*/
                rb.setLiftTarget(0, 30);
                startTime = getRuntime();
                rb.driveTo(-89, 33, 0, 1);
                rb.stop();
                while(getRuntime()<startTime+0.7){
                    rb.lifterHandler();
                }
                rb.driveTo(-77, 19.5, 311, 1);
                rb.turn(270, 1);
                rb.driveTo(-80, 19.5, 270, 1);
                rb.stop();
                rb.setLiftTarget(1, 0);
                while(getRuntime()<startTime+0.7){
                    rb.lifterHandler();
                }
                rb.setLiftTarget(0, 0);
                rb.driveTo(-39, 28, 270, 1);
                rb.stop();
                break;
            default:                /*---------->>  center and unknown  <<----------*/
                rb.driveTo(-5.5, 18, 343, 1);
                rb.grabOpen();
                rb.stop();
                rb.driveTo(-5.5, 29.75, 0, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(-5.5, 25, 0, 1);
                rb.driveTo(-77, 25, 90, 1);
                rb.stop();
                rb.setLiftTarget(1, 100);
                rb.driveTo(-77, 25,0, 1);
                rb.driveTo(-77, 28, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(-77, 23, 0, 1);
                rb.setLiftTarget(0, 45);
                rb.driveTo(18.5, 25, 270, 1);
                rb.driveTo(18.5, 25, 0, 1);
                rb.stop();
                rb.driveTo(18.5, 29.75, 0, 1);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(18.5, 25, 0, 1);
                rb.driveTo(-89, 25, 90, 1);
                rb.stop();
                rb.setLiftTarget(1, 50);
                rb.driveTo(-89, 25, 0, 1);
                rb.driveTo(-89, 31, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                /*rb.setLiftTarget(0, 45);
                rb.driveTo(-89, 25, 0, 1);
                rb.driveTo(-39, 25, 270, 1);
                rb.stop();
                rb.setLiftTarget(0, 0);
                startTime = getRuntime();
                while(getRuntime()<startTime+0.7 && opModeIsActive()){
                    rb.lifterHandler();
                }*/
                rb.setLiftTarget(0, 30);
                startTime = getRuntime();
                rb.driveTo(-89, 33, 0, 1);
                rb.stop();
                while(getRuntime()<startTime+0.7){
                    rb.lifterHandler();
                }
                rb.driveTo(-77, 19.5, 311, 1);
                rb.turn(270, 1);
                rb.driveTo(-80, 19.5, 270, 1);
                rb.stop();
                rb.setLiftTarget(1, 0);
                while(getRuntime()<startTime+0.7){
                    rb.lifterHandler();
                }
                rb.setLiftTarget(0, 0);
                rb.driveTo(-39, 28, 270, 1);
                rb.stop();
                break;
        }
        rb.deconstruct();
    }
}
