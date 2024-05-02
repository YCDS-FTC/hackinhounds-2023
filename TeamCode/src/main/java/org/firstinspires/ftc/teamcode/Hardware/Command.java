package org.firstinspires.ftc.teamcode.Hardware;

public abstract class Command {
    public static final int WAITING = 0;
    public static final int STARTING = 1;
    public static final int RUNNING = 2;
    public static final int ENDING = 3;
    public static final int DONE = 4;

    private int state;
    private Hardware robot;

    public Command() {
        robot = null;
        state = WAITING;
    }

    public Command(Hardware robot) {
        this.robot = robot;
        state = WAITING;
    }

    public int setState(int state) {
        this.state = state;
        return state;
    }

    public int getState() {
        return state;
    }

    public Hardware getRobot() {
        return robot;
    }

    public abstract void init();
    public abstract void run();
    public abstract void end();
}
