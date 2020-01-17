package org.firstinspires.ftc.teamcode.statemachine;

public class DriveStraight implements Command {
    spike_StateMachineRobotBase rb;
    double inches;
    float heading;
    double speedLimit;
    public DriveStraight(double _inches, float _heading, double _speedLimit, spike_StateMachineRobotBase _rb) {
        inches = _inches;
        heading = _heading;
        speedLimit = _speedLimit;
        rb = _rb;
    }

    @Override
    public void init() {
        rb.driveStraight(inches, heading, speedLimit);
    }

    @Override
    public void update() {
        //handled by rb
    }

    @Override
    public void onCompletion() {
        rb.resetDriveStraightVars();
    }

    @Override
    public boolean isFinished() {
        return rb.isDriveStraightFinished();
    }
}
