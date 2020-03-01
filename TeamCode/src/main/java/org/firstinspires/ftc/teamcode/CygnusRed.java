package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="CygnusRed", group="Cygnus")
public class CygnusRed extends LinearOpMode {
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
            case CENTER: //skystone next to wall
                rb.driveTo(-6, 18, 352, 1);
                rb.grabOpen();
                rb.stop();
                rb.driveTo(-6, 29.75, 0, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(-6, 25, 0, 1);
                rb.driveTo(76, 28, 270, 1);
                rb.stop();
                rb.setLiftTarget(1, 100);
                rb.driveTo(76, 28,0, 1);
                rb.stop();
                try {
                    rb.setGlobalY(30 - rb.getDistRInch());
                } catch (IllegalDistanceException e) {
                    rb.setGlobalY(25);
                }
                rb.driveTo(76, 30, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(76, 22, 0, 1);
                rb.dropLifter();
                rb.driveTo(-20.6, 20, 90, 1);
                rb.turn(0, 1);
                rb.stop();
                try {
                    rb.setGlobalY(30 - rb.getDistAverage());
                    rb.driveTo(-23, 31, 322.5f, 0.75, 2);
                    rb.stop();
                    rb.grabClose();
                    sleep(400);
                    rb.driveTo(-18.5, 22, 0, 1);
                    rb.driveTo(89, 22, 270, 1);
                    rb.stop();
                    rb.setLiftTarget(1, 100);
                    rb.driveTo(89, 22, 0, 1);
                    rb.stop();
                    rb.setGlobalY(30 - rb.getDistAverage());
                    rb.driveTo(89, 31, 0, 0.75);
                    rb.stop();
                    rb.grabOpen();
                    sleep(400);
                    rb.dropLifter();
                    startTime = getRuntime();
                    rb.driveTo(89, 35, 0, 1);
                    rb.stop();
                    while (getRuntime() < startTime + 1.0) {
                        rb.lifterHandler();
                    }
                    _globalX = rb.globalX;
                    rb.driveTo(_globalX, 30, 0, 1);
                    rb.pullFoundation(true, 60, 1);
                    rb.turn(90, 1);
                    _globalY = rb.globalY;
                    startTime = getRuntime();
                    rb.setLiftTarget(1, 100);
                    rb.driveTo(82, _globalY, 90, 1, 1);
                    while (getRuntime() < startTime + 0.7) {
                        rb.lifterHandler();
                    }
                    rb.setLiftTarget(0, 0);
                    rb.driveTo(61, 26, 90, 1);
                } catch (IllegalDistanceException e) {
                    telemetry.addData("error!! dist sensor reading", e);
                    telemetry.update();
                    rb.setLiftTarget(0, 0);
                }
                rb.driveTo(36, 26, 90, 1);
                rb.stop();
                break;
            case LEFT: //skystone on end
                rb.driveTo(11, 18, 37, 1);
                rb.grabOpen();
                rb.stop();
                rb.driveTo(11, 30.5, 0, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(11, 25, 0, 1);
                rb.driveTo(76, 28, 270, 1);
                rb.stop();
                rb.setLiftTarget(1, 100);
                rb.driveTo(76, 28,0, 1);
                rb.stop();
                try {
                    rb.setGlobalY(30 - rb.getDistRInch());
                } catch (IllegalDistanceException e) {
                    rb.setGlobalY(25);
                }
                rb.driveTo(76, 30, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(76, 25, 0, 1);
                rb.dropLifter();
                rb.driveTo(-16, 25, 90, 1);
                rb.driveTo(-16, 25, 0, 1);
                rb.stop();
                try {
                    rb.setGlobalY(30 - rb.getDistAverage());
                    rb.driveTo(-16, 30, 0, 1);
                    rb.stop();
                    rb.grabClose();
                    sleep(400);
                    rb.driveTo(-16, 25, 0, 1);
                    rb.driveTo(89, 28, 270, 1);
                    rb.stop();
                    rb.setLiftTarget(1, 100);
                    rb.driveTo(89, 28, 0, 1);
                    rb.stop();
                    rb.setGlobalY(30 - rb.getDistAverage());
                    rb.driveTo(89, 31, 0, 0.75);
                    rb.stop();
                    rb.grabOpen();
                    sleep(400);
                    rb.dropLifter();
                    startTime = getRuntime();
                    rb.driveTo(89, 35, 0, 1);
                    rb.stop();
                    while (getRuntime() < startTime + 1.0) {
                        rb.lifterHandler();
                    }
                    _globalX = rb.globalX;
                    rb.driveTo(_globalX, 30, 0, 1);
                    rb.pullFoundation(true, 60, 1);
                    rb.turn(90, 1);
                    _globalY = rb.globalY;
                    startTime = getRuntime();
                    rb.setLiftTarget(1, 100);
                    rb.driveTo(82, _globalY, 90, 1, 1);
                    while (getRuntime() < startTime + 0.7) {
                        rb.lifterHandler();
                    }
                    rb.setLiftTarget(0, 0);
                    rb.driveTo(61, 26, 90, 1);
                } catch (IllegalDistanceException e) {
                    telemetry.addData("error!! dist sensor reading", e);
                    telemetry.update();
                    rb.setLiftTarget(0, 0);
                }
                rb.driveTo(36, 26, 90, 1);
                rb.stop();
                break;
            default:                /*---------->>  right and unknown  <<----------*/
                rb.driveTo(2, 18, 17, 1);
                rb.grabOpen();
                rb.stop();
                rb.driveTo(2, 29.75, 0, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(2, 25, 0, 1);
                rb.driveTo(76, 28, 270, 1);
                rb.stop();
                rb.setLiftTarget(1, 100);
                rb.driveTo(76, 28,0, 1);
                rb.stop();
                try {
                    rb.setGlobalY(30 - rb.getDistRInch());
                } catch (IllegalDistanceException e) {
                    rb.setGlobalY(25);
                }
                rb.driveTo(76, 30, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(76, 25, 0, 1);
                rb.dropLifter();
                rb.driveTo(-24, 25, 90, 1);
                rb.driveTo(-24, 25, 0, 1);
                rb.stop();
                try {
                    rb.setGlobalY(30 - rb.getDistAverage());
                    rb.driveTo(-24, 29.75, 0, 1);
                    rb.stop();
                    rb.grabClose();
                    sleep(400);
                    rb.driveTo(-24, 25, 0, 1);
                    rb.driveTo(89, 28, 270, 1);
                    rb.stop();
                    rb.setLiftTarget(1, 100);
                    rb.driveTo(89, 28, 0, 1);
                    rb.stop();
                    rb.setGlobalY(30 - rb.getDistAverage());
                    rb.driveTo(89, 31, 0, 0.75);
                    rb.stop();
                    rb.grabOpen();
                    sleep(400);
                    rb.dropLifter();
                    startTime = getRuntime();
                    rb.driveTo(89, 35, 0, 1);
                    rb.stop();
                    while (getRuntime() < startTime + 1.0) {
                        rb.lifterHandler();
                    }
                    _globalX = rb.globalX;
                    rb.driveTo(_globalX, 30, 0, 1);
                    rb.pullFoundation(true, 60, 1);
                    rb.turn(90, 1);
                    _globalY = rb.globalY;
                    startTime = getRuntime();
                    rb.setLiftTarget(1, 100);
                    rb.driveTo(82, _globalY, 90, 1, 1);
                    while (getRuntime() < startTime + 0.7) {
                        rb.lifterHandler();
                    }
                    rb.setLiftTarget(0, 0);
                    rb.driveTo(61, 26, 90, 1);
                } catch (IllegalDistanceException e) {
                    telemetry.addData("error!! dist sensor reading", e);
                    telemetry.update();
                    rb.setLiftTarget(0, 0);
                }
                rb.driveTo(36, 26, 90, 1);
                rb.stop();
                break;
        }
        rb.deconstruct();
    }
}
