package org.firstinspires.ftc.teamcode.statemachine;

public interface Command {
    public void init();
    public void update();
    public void onCompletion();
    public boolean isFinished();
}
