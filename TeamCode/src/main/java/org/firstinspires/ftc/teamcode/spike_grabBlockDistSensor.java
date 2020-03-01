package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Disabled
@Autonomous(name="grabDatBlock")
public class spike_grabBlockDistSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        RobotBaseM1 rb = new RobotBaseM1(this);
        rb.pos4();
        waitForStart();
        rb.stopAndReset();
        rb.yeetBlock();
        sleep(5000);
    }
}
