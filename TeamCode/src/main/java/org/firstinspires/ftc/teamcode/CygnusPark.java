package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="CygnusPark", group="Cygnus")
public class CygnusPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RobotBaseCygnus rb = new RobotBaseCygnus(this);
        telemetry.addData("readyfreddie", true);
        telemetry.update();

        waitForStart();

        rb.grabOpen();
        sleep(400);
        rb.driveTo(0, 9, 0, 1);
        rb.stop();
        rb.deconstruct();
    }
}
