package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Thread.yield;

@Autonomous(name="spike_driveStraightOdometer")
public class spike_driveStraightOdometer extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBaseGabriel rb = new RobotBaseGabriel(this);
        waitForStart();
        rb.driveStraight(12, 12, 0,  0.9); //TODO TEST THIS
        rb.stop();
        sleep(10000);
    }
}
