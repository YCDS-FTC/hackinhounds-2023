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
    private String claw;

    public SetClaws(Hardware robot, ElapsedTime timer, double runTime, String claw, double servoPosition) {
        super(robot);
        this.timer = timer;
        this.runTime = runTime*1000;
        this.servoPosition = servoPosition;
        this.robot = (HackinHoundsHardware) getRobot();
        this.useInit = true;
        this.useEnd = true;
        this.claw = claw;
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
                if (claw == "top") {
                    robot.top_claw.setPosition(servoPosition);
                } else if (claw == "bottom"){
                    robot.bottom_claw.setPosition(servoPosition);
                }
            } else {
                setState(ENDING);
            }
        }
    }

    public void end() {
        setState(DONE);
    }

}
