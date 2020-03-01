package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="VisionTest")
public class VisionTestOpMode extends LinearOpMode {
    VisionBaseM1 visionBoi = new VisionBaseM1(this);

    int startX = 400;
    int stopX = 1280;
    int startY = 350;
    int stopY = 420;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        visionBoi.findSkyStone(startX, stopX, startY, stopY);
        sleep(15000);
    }
}
