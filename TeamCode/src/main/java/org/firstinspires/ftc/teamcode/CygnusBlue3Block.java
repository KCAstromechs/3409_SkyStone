package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="CygnusBlue3Block", group="Cygnus")
public class CygnusBlue3Block extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RobotBaseCygnus rb = new RobotBaseCygnus(this);
        VisionBaseCygnus vb = new VisionBaseCygnus(this);
        double _distL = rb.getDistLInchInit(30.0);
        double _distR = rb.getDistRInchInit(30.0);
        double startTime;
        double _globalY;
        double _globalX;
        VisionBaseCygnus.SKYSTONE_POS pos;
        telemetry.addData("RobotBase", "ok");
        telemetry.addData("VisionBase", "ok");
        telemetry.addData("readyfreddie", true);
        telemetry.addData("","");
        telemetry.addData("zRotation", rb.zRotation);
        telemetry.addData("distL", _distL);
        telemetry.addData("distR", _distR);
        telemetry.update();

        waitForStart();
        double goTime = getRuntime();

        pos = vb.findSkyStone(370, 1135, 440, 610, false);
        //split
        switch(pos) {
            case LEFT:
                rb.driveTo(-13.5, 18, 323, 1, true);
                rb.grabOpen();
                rb.driveTo(-13.5, 29.75, 0, 0.75);
                rb.grabClose();
                rb.stop();
                sleep(200);
                rb.driveTo(-13.5, 23, 0, 1, true);
                break;
            case RIGHT:
                rb.driveTo(2.5, 18, 8, 1, true);
                rb.grabOpen();
                rb.driveTo(2.5, 29.75, 0, 1);
                rb.grabClose();
                rb.stop();
                sleep(100);
                rb.driveTo(2.5, 23, 0, 1, true);
                break;
            default:
                rb.driveTo(-7, 18, 343, 1);
                rb.grabOpen();
                rb.driveTo(-7, 29.75, 0, 0.75);
                rb.grabClose();
                rb.stop();
                sleep(100);
                rb.driveTo(-7, 23, 0, 1, true);
                break;
        }
        //rejoin
        rb.driveTo(-89, 25, 90, 1, true);
        rb.setLiftTarget(1, 100);
        rb.driveTo(-89, 25,0, 1);
        rb.stop();
        try {
            rb.setGlobalY(30 - rb.getDistAverage());
        } catch (IllegalDistanceException e){
            rb.setGlobalY(25);
        }
        rb.driveTo(-89, 32, 0, 0.75);
        rb.grabUp();
        rb.stop();

        //turn foundation after first drop

        rb.dropTimer = 0.35;
        rb.liftSpeedLimit = 0.4;
        rb.dropLifter();
        startTime = getRuntime();
        rb.updateDriveMotors(0.7, 0.7, 0.7, 0.7);
        while(getRuntime()<startTime+0.35){
            rb.updateGlobalPosition();
            rb.lifterHandler();

            sleep(10);
        }
        //rb.driveTo(-89, 36, 0, 1);
        rb.stop();
        /*while(getRuntime()<startTime+0.5){
            rb.lifterHandler();
        }*/
        rb.liftSpeedLimit = 0.6;
        rb.dropTimer = 1;
        _globalX = rb.globalX;
        rb.driveTo(_globalX, 30, 0, 1);
        rb.pullTurn(false, 345, 1);
        _globalX = rb.globalX;
        _globalY = rb.globalY;
        rb.driveTo(_globalX+4.65, _globalY-17.4, 345, 1, true);
        try {
            if (rb.getDistAverage() < 4) {
                rb.pushFoundation(false, 300, 1);
                _globalY = rb.globalY;
                startTime = getRuntime();
                rb.setLiftTarget(1, 100);
                rb.driveTo(-82, _globalY, 270, 1, 1);
                while (getRuntime() < startTime + 0.5) {
                    rb.lifterHandler();
                }
            }
        } catch (IllegalDistanceException e){
            rb.stop();
            rb.setLiftTarget(1, 100);
            sleep(500);
        }
        rb.liftSpeedLimit = 0.5;
        rb.dropLifter();
        rb.driveTo(-61, 31, 270, 1, true);
        rb.grabOpen();
        rb.liftSpeedLimit = 0.6;


        //split

        try {
            switch(pos) {
                case LEFT:
                    rb.driveTo(10.5, 25, 270, 1, true);
                    rb.driveTo(10.5, 25, 0, 1);
                    rb.stop();
                    rb.setGlobalY(30 - rb.getDistAverage());
                    rb.driveTo(10.5, 29.75, 0, 1);
                    rb.grabClose();
                    rb.stop();
                    sleep(200);
                    rb.driveTo(10.5, 21.5, 0, 1, true);
                    rb.driveTo(-62, 21.5, 270, 1, true);
                    rb.setLiftTarget(1, 100);
                    rb.driveTo(-85, 23, 270, 1, 0.7);
                    rb.grabUp();
                    rb.stop();
                    rb.dropLifter();
                    rb.driveTo(-62, 23, 270, 1, true);
                    rb.grabOpen();
                    rb.driveTo(-5, 23, 270, 1, true);
                    rb.driveTo(-5, 23, 0, 1);
                    rb.stop();
                    try {
                        rb.setGlobalY(30 - rb.getDistRInch());
                    } catch (IllegalDistanceException e) {
                        rb.setGlobalY(25);
                    }
                    rb.driveTo(-5, 29.75, 0, 1);
                    rb.grabClose();
                    rb.stop();
                    sleep(200);
                    rb.driveTo(-5, 21.5, 0, 1, true);
                    if(getRuntime()<(26+goTime)) {
                        rb.driveTo(-62, 21.5, 270, 1, true);
                        rb.setLiftTarget(2, 100);
                        rb.driveTo(-85, 25, 270, 1, 0.7);
                        rb.grabUp();
                        rb.stop();
                        rb.setLiftTarget(0, 0);
                        rb.driveTo(-62, 22, 270, 1, true);
                    } else {
                        rb.driveTo(-56, 21.5, 270, 1, true);
                    }
                    rb.grabOpen();
                    break;
                case RIGHT:
                    rb.driveTo(17.1, 25, 270, 1);
                    rb.setGlobalY(20);
                    rb.driveTo(17.1, 25, 30, 1);
                    rb.stop();
                    /*rb.driveTo(18.5, 28, 0, 1);*/
                    rb.driveTo(20, 34, 30, 0.75);
                    rb.grabClose();
                    rb.stop();
                    sleep(200);
                    rb.driveTo(13.5, 20, 0, 1, true);

                    rb.driveTo(-62, 21.5, 270, 1, true);
                    rb.setLiftTarget(2, 100);
                    rb.intermediateRange = 10;
                    rb.driveTo(-85, 15, 250, 1, true);
                    rb.intermediateRange = 4;
                    rb.grabUp();
                    rb.driveTo(-95, 15, 270, 1, 0.7);
                    rb.stop();
                    rb.setLiftTarget(0, 0);
                    rb.driveTo(-62, 22, 250, 1, true);
                    rb.grabOpen();
                    /*rb.driveTo(-62, 21.5, 270, 1, true);
                    rb.setLiftTarget(1, 100);
                    rb.driveTo(-85, 23, 270, 1, 1);
                    rb.grabUp();
                    rb.stop();
                    rb.dropLifter();
                    rb.driveTo(-62, 23, 270, 1, true);
                    rb.grabOpen();*/
                    rb.driveTo(-13.5, 23, 270, 1, true);
                    rb.driveTo(-13.5, 23, 0, 1);
                    rb.stop();
                    try {
                        rb.setGlobalY(30 - rb.getDistRInch());
                    } catch (IllegalDistanceException e) {
                        rb.setGlobalY(25);
                    }
                    rb.driveTo(-13.5, 29.75, 0, 1);
                    rb.grabClose();
                    rb.stop();
                    sleep(100);
                    rb.driveTo(-13.5, 21.5, 0, 1, true);
                    rb.driveTo(-56, 21.5, 270, 1, true);
                    rb.grabOpen();
                    break;
                default:
                    rb.driveTo(18.5, 25, 270, 1, true);
                    rb.driveTo(18.5, 25, 0, 1);
                    rb.stop();
                    rb.setGlobalY(30 - rb.getDistAverage());
                    rb.driveTo(18.5, 29.75, 0, 1);
                    rb.grabClose();
                    rb.stop();
                    sleep(100);
                    rb.driveTo(18.5, 21.5, 0, 1, true);
                    rb.driveTo(-62, 21.5, 270, 1, true);
                    rb.setLiftTarget(1, 100);
                    rb.driveTo(-85, 23, 270, 1, 1);
                    rb.grabUp();
                    rb.stop();
                    rb.dropLifter();
                    rb.driveTo(-62, 23, 270, 1, true);
                    rb.grabOpen();
                    rb.driveTo(2.5, 23, 270, 1, true);
                    rb.driveTo(2.5, 23, 0, 1);
                    rb.stop();
                    try {
                        rb.setGlobalY(30 - rb.getDistRInch());
                    } catch (IllegalDistanceException e) {
                        rb.setGlobalY(25);
                    }
                    rb.driveTo(2.5, 29.75, 0, 1);
                    rb.grabClose();
                    rb.stop();
                    sleep(100);
                    rb.driveTo(2.5, 21.5, 0, 1, true);
                    if(getRuntime()<(25+goTime)) {
                        rb.driveTo(-62, 21.5, 270, 1, true);
                        rb.setLiftTarget(2, 100);
                        rb.intermediateRange = 10;
                        rb.driveTo(-85, 15, 250, 1, true);
                        rb.intermediateRange = 4;
                        rb.grabUp();
                        rb.driveTo(-95, 15, 270, 1, 0.7);
                        rb.stop();
                        rb.setLiftTarget(0, 0);
                        rb.driveTo(-62, 22, 250, 1, true);
                    } else {
                        rb.driveTo(-56, 21.5, 270, 1, true);
                    }
                    rb.grabOpen();
                    break;
            }
        } catch (IllegalDistanceException e) {
            telemetry.addData("error!! dist sensor reading", e);
            telemetry.update();
            rb.setLiftTarget(0, 0);
        }
        rb.driveTo(-36, 25, 270, 1);
        rb.stop();
        rb.deconstruct();
    }
}
