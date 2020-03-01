package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Thread.yield;

@Disabled
@Autonomous(name="spike_driveStraightOdometer")
public class spike_driveStraightOdometer extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBaseCygnus rb = new RobotBaseCygnus(this);
        VisionBaseCygnus vb = new VisionBaseCygnus(this);
        telemetry.addData("readyfreddie", true);
        telemetry.update();
        waitForStart();
        rb.pullFoundation(true, 45, 1);
        rb.turn(90,1);
        double _globalY = rb.globalY;
        rb.driveTo(-9, _globalY, 90, 1, 1);
        rb.stop();
        while(opModeIsActive()){
            telemetry.addData("zRotation", rb.zRotation);
            telemetry.update();
        }
        rb.deconstruct();
    }
}
