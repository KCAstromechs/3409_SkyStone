package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="CygnusRed")
public class CygnusRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraPos;

        RobotBaseCygnus rb = new RobotBaseCygnus(this);
        VisionBaseCygnus vb = new VisionBaseCygnus(this);
        telemetry.addData("ready", true);
        telemetry.update();

        waitForStart();

        switch (vb.findSkyStone(430, 1170, 330, 470, false)){
            case CENTER: //skystone next to wall
                rb.driveTo(-7.5, 18, 352, 1);
                rb.grabOpen();
                rb.stop();
                rb.driveTo(-7.5, 32, 0, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(-7.5, 27, 0, 1);
                rb.driveTo(89, 27, 270, 1);
                rb.stop();
                rb.setLiftTarget(1, 100);
                rb.driveTo(89, 27,0, 1);
                rb.driveTo(89, 36, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(89, 27, 0, 1);
                rb.setLiftTarget(0, 45);
                rb.driveTo(-21, 27, 90, 1);
                rb.driveTo(-21, 27, 320, 1);
                rb.stop();
                rb.driveTo(-24, 36, 320, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(-18.5, 27, 320, 1);
                rb.driveTo(89, 27, 270, 1);
                rb.stop();
                rb.setLiftTarget(2, 0);
                rb.driveTo(89, 27, 0, 1);
                rb.driveTo(89, 38.5, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.setLiftTarget(0, 45);
                rb.driveTo(89, 27, 0, 1);
                rb.driveTo(39, 27, 90, 1);
                rb.stop();
                rb.setLiftTarget(0, 0);
                double startTime = getRuntime();
                while(getRuntime()<startTime+1.7 && opModeIsActive()){
                    rb.lifterHandler();
                }
                break;
            case LEFT: //skystone on end
                rb.driveTo(11, 18, 31.5f, 1);
                rb.grabOpen();
                rb.stop();
                rb.driveTo(11, 32, 0, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(11, 27, 0, 1);
                rb.driveTo(89, 27, 270, 1);
                rb.stop();
                rb.setLiftTarget(1, 100);
                rb.driveTo(89, 27,0, 1);
                rb.driveTo(89, 36, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(89, 27, 0, 1);
                rb.setLiftTarget(0, 45);
                rb.driveTo(-13, 27, 270, 1);
                rb.driveTo(-13, 27, 0, 1);
                rb.stop();
                rb.driveTo(-13, 30, 0, 1);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(-13, 27, 0, 1);
                rb.driveTo(89, 27, 270, 1);
                rb.stop();
                rb.setLiftTarget(2, 0);
                rb.driveTo(89, 27, 0, 1);
                rb.driveTo(89, 37, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.setLiftTarget(0, 45);
                rb.driveTo(89, 27, 0, 1);
                rb.driveTo(39, 27, 90, 1);
                rb.stop();
                rb.setLiftTarget(0, 0);
                startTime = getRuntime();
                while(getRuntime()<startTime+1.7 && opModeIsActive()){
                    rb.lifterHandler();
                }
                break;
            default:                /*---------->>  right and unknown  <<----------*/
                rb.driveTo(2, 18, 7, 1);
                rb.grabOpen();
                rb.stop();
                rb.driveTo(2, 32, 0, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(2, 27, 0, 1);
                rb.driveTo(89, 27, 270, 1);
                rb.stop();
                rb.setLiftTarget(1, 100);
                rb.driveTo(89, 27,0, 1);
                rb.driveTo(89, 36, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(89, 26, 0, 1);
                rb.setLiftTarget(0, 45);
                rb.driveTo(-22, 27, 90, 1);
                rb.driveTo(-22, 27, 0, 1);
                rb.stop();
                rb.driveTo(-22, 30, 0, 1);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(-22, 27, 0, 1);
                rb.driveTo(89, 27, 270, 1);
                rb.stop();
                rb.setLiftTarget(2, 0);
                rb.driveTo(89, 27, 0, 1);
                rb.driveTo(89, 37, 0, 0.75);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.setLiftTarget(0, 45);
                rb.driveTo(89, 27, 0, 1);
                rb.driveTo(39, 27, 90, 1);
                rb.stop();
                rb.setLiftTarget(0, 0);
                startTime = getRuntime();
                while(getRuntime()<startTime+1.7 && opModeIsActive()){
                    rb.lifterHandler();
                }
                break;
        }
        rb.deconstruct();
    }
}
