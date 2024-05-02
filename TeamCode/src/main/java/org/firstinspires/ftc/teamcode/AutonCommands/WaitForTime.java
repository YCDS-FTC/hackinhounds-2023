package org.firstinspires.ftc.teamcode.AutonCommands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Command;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class WaitForTime extends Command {
    private ElapsedTime timer;
    private double startTime;
    private double runTime;
    private HackinHoundsHardware robot;

    public WaitForTime(Hardware robot, ElapsedTime timer, double runTime) {
        super(robot);
        this.timer = timer;
        this.runTime = runTime*1000;
        this.robot = (HackinHoundsHardware) getRobot();
        setState(STARTING);
    }

    public void init() {
        if (getState() == STARTING) {
            startTime = timer.milliseconds();
            setState(RUNNING);
        }
    }

    public void run() {
        if (getState() == RUNNING) {
            double elapsedTime = timer.milliseconds()-startTime;
            if (elapsedTime > runTime) {
                setState(ENDING);
            }
        }
    }

    public void end() {
        setState(DONE);
    }

}
