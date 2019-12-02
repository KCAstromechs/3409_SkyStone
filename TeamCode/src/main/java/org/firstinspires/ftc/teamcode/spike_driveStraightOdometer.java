package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Thread.yield;

@Autonomous(name="spike_driveStraightOdometer")
public class spike_driveStraightOdometer extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBaseM1 rb = new RobotBaseM1(this);
        waitForStart();
        rb.driveStraightOdometerSpike(70, 0, 0.9);
        rb.stop();
        telemetry.addData("e", rb.getEncoderWheelPosition());
        telemetry.update();
        sleep(10000);
    }
}
