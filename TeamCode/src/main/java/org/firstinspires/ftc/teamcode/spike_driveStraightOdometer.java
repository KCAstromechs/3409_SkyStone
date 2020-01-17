package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Thread.yield;

@Autonomous(name="spike_driveStraightOdometer")
public class spike_driveStraightOdometer extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBaseCygnus rb = new RobotBaseCygnus(this);
        VisionBaseCygnus vb = new VisionBaseCygnus(this);
        waitForStart();
        vb.findSkyStone(1, 2, 3, 4, true);
        telemetry.addData("position", vb.findSkyStone(430, 1170, 330, 470, true));
        telemetry.update();
        rb.stop();
        while(opModeIsActive());
        rb.deconstruct();
    }
}
