package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="spike_distanceSensorTest")
public class spike_distanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBaseCygnus rb = new RobotBaseCygnus(this);
        VisionBaseCygnus vb = new VisionBaseCygnus(this);

        telemetry.addData("readyfreddie", true);
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("zRotation", rb.zRotation);
            try {
                telemetry.addData("distL", rb.getDistLInch());
            } catch (IllegalDistanceException e) {
                telemetry.addData("error!! distL", e);
            }
            try {
                telemetry.addData("distR", rb.getDistRInch());
            } catch (IllegalDistanceException e) {
                telemetry.addData("error!! distR", e);
            }
            telemetry.update();
            sleep(200);
        }
        rb.deconstruct();
    }
}
