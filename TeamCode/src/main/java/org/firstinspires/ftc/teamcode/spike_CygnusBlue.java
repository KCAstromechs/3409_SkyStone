package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="spike_CygnusBlue", group="Cygnus")
public class spike_CygnusBlue extends LinearOpMode {
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

        rb.driveTo(-13.5, 18, 323, 1, true);
        rb.grabOpen();
        rb.driveTo(-13.5, 29.75, 0, 0.75);
        rb.stop();
        rb.grabClose();
        sleep(400);
        rb.driveTo(-13.5, 25, 0, 1, true);
        rb.driveTo(-78, 25, 90, 1, true);
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
        rb.driveTo(-78, 23, 0, 1, true);
        rb.dropLifter();
        rb.driveTo(10.5, 25, 270, 1, true);
        rb.driveTo(10.5, 25, 0, 1);
        rb.stop();
        rb.stop();
        rb.deconstruct();
    }
}
