package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DistanceSensor;

        import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="spike_foundationGrab")
public class spike_foundationGrab extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBaseCygnus rb = new RobotBaseCygnus(this);
        VisionBaseCygnus vb = new VisionBaseCygnus(this);

        double startTime;
        double _globalY;
        double _globalX;

        telemetry.addData("readyfreddie", true);
        telemetry.update();
        waitForStart();
        rb.pullTurn(true, 15, 1);
        _globalX = rb.globalX;
        _globalY = rb.globalY;
        rb.driveTo(_globalX-4.65, _globalY-17.4, 15, 1, true);
        rb.pushFoundation(true, 60, 1);
        _globalY = rb.globalY;
        startTime = getRuntime();
        rb.setLiftTarget(1, 100);
        rb.driveTo(82, _globalY, 90, 1, 0.5);
        while(getRuntime()<startTime+0.7){
            rb.lifterHandler();
        }
        rb.stop();
        rb.deconstruct();
    }
}
