package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="CygnusBlue", group="Cygnus")
public class CygnusBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RobotBaseCygnus rb = new RobotBaseCygnus(this);
        VisionBaseCygnus vb = new VisionBaseCygnus(this);
        double _distL = rb.getDistLInchInit(30.0);
        double _distR = rb.getDistRInchInit(30.0);
        double startTime;
        double _globalY;
        double _globalX;
        telemetry.addData("RobotBase", "ok");
        telemetry.addData("VisionBase", "ok");
        telemetry.addData("readyfreddie", true);
        telemetry.addData("","");
        telemetry.addData("zRotation", rb.zRotation);
        telemetry.addData("distL", _distL);
        telemetry.addData("distR", _distR);
        telemetry.update();

        waitForStart();

        switch (vb.findSkyStone(370, 1135, 440, 610, false)){
            case LEFT:
                rb.driveTo(-13.5, 18, 323, 1);
                rb.grabOpen();
                rb.stop();
                rb.driveTo(-13.5, 29.75, 0, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(-13.5, 25, 0, 1);
                rb.driveTo(-78, 25, 90, 1);
                rb.stop();
                rb.setLiftTarget(1, 100);
                rb.driveTo(-78, 25,0, 1);
                rb.stop();
                try {
                    rb.setGlobalY(30 - rb.getDistLInch());
                } catch (IllegalDistanceException e){
                    rb.setGlobalY(25);
                }
                rb.driveTo(-78, 30, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(-78, 25, 0, 1);
                rb.dropLifter();
                rb.driveTo(10.5, 25, 270, 1);
                rb.driveTo(10.5, 25, 0, 1);
                rb.stop();
                try {
                    rb.setGlobalY(30-rb.getDistAverage());
                    rb.driveTo(10.5, 29.75, 0, 1);
                    rb.stop();
                    rb.grabClose();
                    sleep(400);
                    rb.driveTo(10.5, 25, 0, 1);
                    rb.driveTo(-89, 25, 90, 1);
                    rb.stop();
                    rb.setLiftTarget(1, 100);
                    rb.driveTo(-89, 25, 0, 1);
                    rb.stop();
                    rb.setGlobalY(30-(rb.getDistAverage()));
                    rb.driveTo(-89, 31, 0, 0.75);
                    rb.stop();
                    rb.grabOpen();
                    sleep(400);
                    rb.dropLifter();
                    startTime = getRuntime();
                    rb.driveTo(-89, 35, 0, 1);
                    rb.stop();
                    while(getRuntime()<startTime+1.0){
                        rb.lifterHandler();
                    }
                    _globalX = rb.globalX;
                    rb.driveTo(_globalX, 30, 0, 1);
                    rb.pullFoundation(false, 300, 1);
                    rb.turn(270,1);
                    _globalY = rb.globalY;
                    startTime = getRuntime();
                    rb.setLiftTarget(1, 100);
                    rb.driveTo(-82, _globalY, 270, 1, 1);
                    while(getRuntime()<startTime+0.7){
                        rb.lifterHandler();
                    }
                    rb.setLiftTarget(0, 0);
                    rb.driveTo(-61, 28, 270, 1);
                } catch (IllegalDistanceException e) {
                    telemetry.addData("error!! dist sensor reading", e);
                    telemetry.update();
                    rb.setLiftTarget(0, 0);
                }
                rb.driveTo(-36, 28, 270, 1);
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
                rb.driveTo(-78, 25, 90, 1);
                rb.stop();
                rb.setLiftTarget(1, 100);
                rb.driveTo(-78, 25,0, 1);
                rb.stop();
                try {
                    rb.setGlobalY(30 - rb.getDistLInch());
                } catch (IllegalDistanceException e){
                    rb.setGlobalY(25);
                }
                rb.driveTo(-78, 30, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(-78, 23, 0, 1);
                rb.dropLifter();
                rb.driveTo(/*18.5*/17.1, 25, 270, 1);
                rb.setGlobalY(15);
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
                rb.setLiftTarget(1, 100);
                rb.driveTo(-89, 25, 0, 1);
                rb.stop();
                try {
                    rb.setGlobalY(30 - rb.getDistAverage());
                    rb.driveTo(-89, 31, 0, 0.75);
                    rb.stop();
                    rb.grabOpen();
                    sleep(400);
                    rb.dropLifter();
                    startTime = getRuntime();
                    rb.driveTo(-89, 35, 0, 1);
                    rb.stop();
                    while (getRuntime() < startTime + 1.0) {
                        rb.lifterHandler();
                    }
                    _globalX = rb.globalX;
                    rb.driveTo(_globalX, 30, 0, 1);
                    rb.pullFoundation(false, 300, 1);
                    rb.turn(270, 1);
                    _globalY = rb.globalY;
                    startTime = getRuntime();
                    rb.setLiftTarget(1, 100);
                    rb.driveTo(-82, _globalY, 270, 1, 1);
                    while (getRuntime() < startTime + 0.7) {
                        rb.lifterHandler();
                    }
                    rb.setLiftTarget(0, 0);
                    rb.driveTo(-61, 28, 270, 1);
                } catch (IllegalDistanceException e) {
                    telemetry.addData("error!! dist sensor reading", e);
                    telemetry.update();
                    rb.setLiftTarget(0, 0);
                }
                rb.driveTo(-36, 28, 270, 1);
                rb.stop();
                break;
            default:                /*---------->>  center and unknown  <<----------*/
                rb.driveTo(-7, 18, 343, 1);
                rb.grabOpen();
                rb.stop();
                rb.driveTo(-7, 29.75, 0, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(-7, 25, 0, 1);
                rb.driveTo(-78, 25, 90, 1);
                rb.stop();
                rb.setLiftTarget(1, 100);
                rb.driveTo(-78, 25,0, 1);
                rb.stop();
                try {
                    rb.setGlobalY(30 - rb.getDistLInch());
                } catch (IllegalDistanceException e){
                    rb.setGlobalY(25);
                }
                rb.driveTo(-78, 30, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(-78, 23, 0, 1);
                rb.dropLifter();
                rb.driveTo(18.5, 25, 270, 1);
                rb.driveTo(18.5, 25, 0, 1);
                rb.stop();
                try {
                    rb.setGlobalY(30 - ((rb.getDistLInch() + rb.getDistRInch()) / 2));
                    rb.driveTo(18.5, 29.75, 0, 1);
                    rb.stop();
                    rb.grabClose();
                    sleep(400);
                    rb.driveTo(18.5, 25, 0, 1);
                    rb.driveTo(-89, 25, 90, 1);
                    rb.stop();
                    rb.setLiftTarget(1, 100);
                    rb.driveTo(-89, 25, 0, 1);
                    rb.stop();
                    rb.setGlobalY(30 - ((rb.getDistLInch() + rb.getDistRInch()) / 2));
                    rb.driveTo(-89, 31, 0, 0.75);
                    rb.stop();
                    rb.grabOpen();
                    sleep(400);
                    rb.dropLifter();
                    startTime = getRuntime();
                    rb.driveTo(-89, 35, 0, 1);
                    rb.stop();
                    while (getRuntime() < startTime + 1.0) {
                        rb.lifterHandler();
                    }
                    _globalX = rb.globalX;
                    rb.driveTo(_globalX, 30, 0, 1);
                    rb.pullFoundation(false, 300, 1);
                    rb.turn(270, 1);
                    _globalY = rb.globalY;
                    startTime = getRuntime();
                    rb.setLiftTarget(1, 100);
                    rb.driveTo(-82, _globalY, 270, 1, 1);
                    while (getRuntime() < startTime + 0.7) {
                        rb.lifterHandler();
                    }
                    rb.setLiftTarget(0, 0);
                    rb.driveTo(-61, 28, 270, 1);
                } catch (IllegalDistanceException e) {
                    telemetry.addData("error!! dist sensor reading", e);
                    telemetry.update();
                    rb.setLiftTarget(0, 0);
                }
                rb.driveTo(-36, 28, 270, 1);
                rb.stop();
                break;
        }
        rb.deconstruct();
    }
}
