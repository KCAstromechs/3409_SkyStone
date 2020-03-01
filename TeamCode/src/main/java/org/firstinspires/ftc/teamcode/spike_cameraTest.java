package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="spike_cameraTest")
public class spike_cameraTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBaseCygnus rb = new RobotBaseCygnus(this);
        VisionBaseCygnus vb = new VisionBaseCygnus(this);
        waitForStart();
        vb.findSkyStone(1, 2, 3, 4, true);
        telemetry.addData("position", vb.findSkyStone(370, 1135, 440, 610, true));
        telemetry.update();
        rb.stop();
        while(opModeIsActive());
        rb.deconstruct();
    }
}
