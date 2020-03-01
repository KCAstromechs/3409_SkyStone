package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="CygnusBlueNoLift", group="Cygnus")
public class CygnusBlueNoLift extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RobotBaseCygnus rb = new RobotBaseCygnus(this);
        VisionBaseCygnus vb = new VisionBaseCygnus(this);
        telemetry.addData("readyfreddie", true);
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
                rb.grabOpen();
                sleep(400);
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
                rb.grabOpen();
                sleep(400);
                rb.driveTo(-61, 34, 270, 1);
                rb.driveTo(-36, 34, 270, 1);
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
                rb.grabOpen();
                sleep(400);
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
                rb.grabOpen();
                sleep(400);
                rb.driveTo(-61, 34, 270, 1);
                rb.driveTo(-36, 34, 270, 1);
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
                rb.driveTo(-78, 25, 90, 1);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(18.5, 25, 270, 1);
                rb.driveTo(18.5, 25, 0, 1);
                rb.stop();
                rb.setGlobalY(18);
                rb.driveTo(18.5, 29.75, 0, 1);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(18.5, 25, 0, 1);
                rb.driveTo(-89, 25, 90, 1);
                rb.stop();
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(-61, 34, 270, 1);
                rb.driveTo(-36, 34, 270, 1);
                rb.stop();
                break;
        }
        rb.deconstruct();
    }
}
