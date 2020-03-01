package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="Autonomous Testing Environment", group="zzzz")
public class AutonomousTestingEnvironment extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RobotBaseCygnus rb = new RobotBaseCygnus(this);
        VisionBaseCygnus vb = new VisionBaseCygnus(this);
        telemetry.addData("readyfreddie", true);
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("getRuntime", getRuntime());
            telemetry.update();
        }
        rb.deconstruct();
    }
}
