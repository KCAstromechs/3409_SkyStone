package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="CygnusRedNoLift", group="Cygnus")
public class CygnusRedNoLift extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RobotBaseCygnus rb = new RobotBaseCygnus(this);
        VisionBaseCygnus vb = new VisionBaseCygnus(this);
        telemetry.addData("readyfreddie", true);
        telemetry.update();

        waitForStart();

        switch (vb.findSkyStone(370, 1135, 440, 610, false)){
            case CENTER: //skystone next to wall
                rb.driveTo(-5.5, 18, 352, 1);
                rb.grabOpen();
                rb.stop();
                rb.driveTo(-5.5, 29.75, 0, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(-5.5, 25, 0, 1);
                rb.driveTo(76, 28, 270, 1);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(-20.6, 25, 90, 1);
                rb.setGlobalY(15);
                rb.driveTo(-20.6, 25, 330, 1);
                rb.stop();
                rb.driveTo(-23.5, 34, 330, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(-18.5, 25, 0, 1);
                rb.driveTo(89, 28, 270, 1);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(61, 34, 90, 1);
                rb.driveTo(36, 34, 90, 1);
                rb.stop();
                break;
            case LEFT: //skystone on end
                rb.driveTo(11, 18, 37, 1);
                rb.grabOpen();
                rb.stop();
                rb.driveTo(11, 29.75, 0, 0.75);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(11, 25, 0, 1);
                rb.driveTo(76, 28, 270, 1);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(76, 25, 0, 1);
                rb.setLiftTarget(0, 45);
                rb.driveTo(-14, 25, 270, 1);
                rb.driveTo(-14, 25, 0, 1);
                rb.stop();
                rb.driveTo(-14, 30, 0, 1);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(-14, 25, 0, 1);
                rb.driveTo(89, 28, 270, 1);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(61, 34, 90, 1);
                rb.driveTo(36, 34, 90, 1);
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
                rb.grabOpen();
                sleep(400);
                rb.driveTo(-22, 25, 90, 1);
                rb.driveTo(-22, 25, 0, 1);
                rb.stop();
                rb.setGlobalY(19);
                rb.driveTo(-22, 29.75, 0, 1);
                rb.stop();
                rb.grabClose();
                sleep(400);
                rb.driveTo(-22, 25, 0, 1);
                rb.driveTo(89, 28, 270, 1);
                rb.stop();
                rb.grabOpen();
                sleep(400);
                rb.driveTo(61, 34, 90, 1);
                rb.driveTo(36, 34, 90, 1);
                rb.stop();
                break;
        }
        rb.deconstruct();
    }
}
