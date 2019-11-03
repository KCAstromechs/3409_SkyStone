package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="encoderTest")
public class encoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RobotBaseM1 robotBase = new RobotBaseM1(this);

        waitForStart();

        while(true){
            telemetry.addData("frontRight", robotBase.getCurrentFrontRightPosition());
            telemetry.addData("frontLeft", robotBase.getCurrentFrontLeftPosition());
            telemetry.addData("backRight", robotBase.getCurrentBackRightPosition());
            telemetry.addData("backLeft", robotBase.getCurrentBackLeftPosition());
            telemetry.update();
            sleep(100);
        }
    }
}