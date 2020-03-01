package org.firstinspires.ftc.teamcode.statemachine;

import java.util.ArrayList;
import java.util.LinkedList;

public class CommandHandler {
    Command currentCommand;
    ArrayList<Command> commandList;
    boolean isActive = true;
    public CommandHandler() {
         commandList = new ArrayList<>();
    }
    public void start() {
        currentCommand = commandList.get(0);
        currentCommand.init();
    }
    public void update() {
        if(isActive) {
            currentCommand.update();
            if(currentCommand.isFinished()) {
                currentCommand.onCompletion();
                if(commandList.indexOf(currentCommand) < commandList.size() - 1) {
                    currentCommand = commandList.get(commandList.indexOf(currentCommand) + 1);
                }
                else {
                    isActive = false;
                }
                currentCommand.init();
            }
        }
    }
    public void addSequential(Command c) {
        commandList.add(c);
    }
    public void addParallel(LinkedList<Command> cl) {
        //commandList.add(new ParallelCommand(cl));
    }

}
