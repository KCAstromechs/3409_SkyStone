package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="VisionTest")
public class VisionTestOpMode extends LinearOpMode {
    VisionBase visionBoi = new VisionBase(this);
    int blueStartX = 180;
    int blueStopX = 1040;
    int blueStartY = 350;
    int blueStopY = 420;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        visionBoi.findSkyStone(blueStartX, blueStopX, blueStartY, blueStartY, true);
        sleep(20000);
    }
}
