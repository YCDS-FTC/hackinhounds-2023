package org.firstinspires.ftc.teamcode.AutonCommands;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Command;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class SetWrist extends Command {
    private ElapsedTime timer;
    private double startTime;
    private double runTime;
    private HackinHoundsHardware robot;
    private double servoPosition;
    private boolean useInit;
    private boolean useEnd;

    public SetWrist(Hardware robot, ElapsedTime timer, double runTime, double servoPosition) {
        super(robot);
        this.timer = timer;
        this.runTime = runTime*1000;
        this.servoPosition = servoPosition;
        this.robot = (HackinHoundsHardware) getRobot();
        this.useInit = true;
        this.useEnd = true;
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
                robot.wrist.setPosition(servoPosition);
            } else {
                setState(ENDING);
            }
        }
    }

    public void end() {
        setState(DONE);
    }

}
