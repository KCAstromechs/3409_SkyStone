package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous (name="GabrielAscendedBlue")
public class GabrielAscendedBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraPos;

        RobotBaseGabriel rb = new RobotBaseGabriel(this);
        VisionBaseGabriel vb = new VisionBaseGabriel(this);

        waitForStart();

        /*switch (vb.findSkyStone(400, 1280, 350, 420, false)){
            case LEFT:
                cameraPos = 0;
                break;
            case RIGHT:
                cameraPos = 2;
                break;
            default:                *//*---------->>  center and unknown  <<----------*//*
                cameraPos = 1;
                break;
        }*/ cameraPos = 0;
        switch(cameraPos) {
            case 0:
                rb.driveTo(11, -18, 180, 0.9);
                rb.stop();
                rb.pos4();
                rb.driveTo(11, -28, 180, 0.6);
                rb.stop();
                rb.pos5();
                rb.pos3();
                rb.driveTo(11, -18, 180, 0.9);
                rb.driveTo(88, -18, 270, 0.9);
                rb.stop();
                rb.lift1F();
                rb.driveTo(88, -18, 180, 0.9);
                rb.driveTo(88, -28, 180, 0.6);
                rb.stop();
                rb.pos5();
                rb.pos4();
                rb.driveTo(88, -18, 180, 0.9);
                rb.driveTo(-15, -18, 90, 0.9);
                rb.driveTo(-15, -18, 180, 0.9);
                rb.driveTo(-15, -28, 180, 0.9);
                rb.stop();
                rb.pos5();
                rb.pos3();
                rb.driveTo(-15, -18, 180, 0.9);
                rb.driveTo(88, -18, 270, 0.9);
                rb.stop();
                rb.lift2F();
                rb.driveTo(88, -18, 180, 0.9);
                rb.driveTo(88, -28, 180, 0.6);
                rb.stop();
                rb.pos5();
                rb.pos4();
                break;
            case 1:
                rb.driveTo(4, -17, 180, 0.9);
                break;
            case 2:
                rb.driveTo(-4, -17, 180, 0.9);
                break;
        }
        rb.deconstruct();
    }
}
