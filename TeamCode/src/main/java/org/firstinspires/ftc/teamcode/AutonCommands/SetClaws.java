package org.firstinspires.ftc.teamcode.AutonCommands;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Command;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class SetClaws extends Command {
    private ElapsedTime timer;
    private double startTime;
    private double runTime;
    private HackinHoundsHardware robot;
    private double servoPosition;
    private boolean useInit;
    private boolean useEnd;

    public SetClaws(Hardware robot, ElapsedTime timer, double runTime, double servoPosition) {
        super(robot);
        this.timer = timer;
        this.runTime = runTime*1000;
        this.servoPosition = servoPosition;
        this.robot = (HackinHoundsHardware) getRobot();
        this.useInit = true;
        this.useEnd = true;
        setState(STARTING);
    }

    public SetClaws(Hardware robot, ElapsedTime timer, double runTime, double servoPosition, boolean useInit, boolean useEnd) {
        super(robot);
        this.timer = timer;
        this.runTime = runTime*1000;
        this.servoPosition = servoPosition;
        this.robot = (HackinHoundsHardware) getRobot();
        this.useInit = useInit;
        this.useEnd = useEnd;
        setState(STARTING);
    }

    public void init() {
        startTime = timer.milliseconds();
        setState(RUNNING);
    }

    public void run() {
        if (getState() == RUNNING) {
            double elapsedTime = timer.milliseconds()-startTime;
            if (elapsedTime < runTime) {
                robot.top_claw.setPosition(servoPosition);
            } else {
                setState(ENDING);
            }
        }
    }

    public void end() {
        setState(DONE);
    }

}
