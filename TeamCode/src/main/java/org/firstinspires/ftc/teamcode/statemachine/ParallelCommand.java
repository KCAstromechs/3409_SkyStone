package org.firstinspires.ftc.teamcode.statemachine;

import org.firstinspires.ftc.teamcode.statemachine.Command;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.LinkedList;

public class ParallelCommand implements Command {
    ArrayList<Command> list = new ArrayList<>();
    ArrayList<Boolean> isFinished = new ArrayList<>();
    boolean isFinishedTracker = false;

    public ParallelCommand(ArrayList<Command> _list) {
        list = _list;
    }
    @Override
    public void init() {
        for (Command i:list) {
            i.init();
        }
    }

    @Override
    public void update() {
        for(Command i:list) {
            i.update();
        }
    }

    @Override
    public void onCompletion() {
        for(Command i:list) {
            i.onCompletion();
        }
    }

    @Override
    public boolean isFinished() {
        isFinishedTracker = list.get(0).isFinished();
        for(int i = 0; i < list.size() - 1; i++) {
            isFinishedTracker = isFinishedTracker && list.get(i+1).isFinished();
        }
        return isFinishedTracker;
    }
}
