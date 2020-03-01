package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
@Autonomous (name="State machine test")
public class spike_stateMachineOpMode extends OpMode {

    spike_StateMachineRobotBase rb;
    CommandHandler ch;
    @Override
    public void init() {
        rb = new spike_StateMachineRobotBase(this);
        ch = new CommandHandler();
        ch.addSequential(new DriveStraight(24, 0 , 1, rb));
        ch.start();
    }

    @Override
    public void loop() {
        rb.updateDriveTrain();
        ch.update();
    }

    public void stop() {

    }

}
