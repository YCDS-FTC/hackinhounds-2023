package org.firstinspires.ftc.teamcode.AutonCommands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Command;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import java.util.ArrayList;
import java.util.List;

public class TestForOthers extends Command {
    private ElapsedTime timer;
    private double startTime;
    private double timeOut;
    private HackinHoundsHardware robot;

    private int leftFrontTicks;
    private int rightFrontTicks;
    private int leftFrontStartTicks;
    private int rightFrontStartTicks;
    private int leftEndTicks;
    private int rightEndTicks;
    private double RepositionDistance1;
    private double RepositionDistance2;
    private double RepositionDistance3;
    private double deadReckonDistance1;
    private double deadReckonDistance2;
    private double deadReckonDistance3;
    private double measure;
    private int slidePosition;
    private double powerLevel;
    private double sPowerLevel;
    private double prop;
    private boolean slideDone = false;

    public TestForOthers(Hardware robot, ElapsedTime timer, double RepositionDistance1, double RepositionDistance2, double RepositionDistance3, double deadReckonDistance1, double deadReckonDistance2, double deadReckonDistance3, double timeOut, double powerLevel, int slidePosition, double sPowerLevel, int prop) {
        super(robot);
        this.timer = timer;
        this.RepositionDistance1 = RepositionDistance1;
        this.RepositionDistance2 = RepositionDistance2;
        this.RepositionDistance3 = RepositionDistance3;
        this.deadReckonDistance1 = deadReckonDistance1;
        this.deadReckonDistance2 = deadReckonDistance2;
        this.deadReckonDistance3 = deadReckonDistance3;
        this.slidePosition = slidePosition;
        this.sPowerLevel = sPowerLevel;
        this.powerLevel = powerLevel;
        this.prop = prop;
        if (timeOut<0){
            timeOut=30;
        }
        this.timeOut = timeOut*1000;
        this.robot = (HackinHoundsHardware) getRobot();
        leftFrontTicks=0;
        rightFrontTicks=0;
        setState(STARTING);
    }

    public void init() {
        if (getState() == STARTING) {
            setState(RUNNING);
            startTime = timer.milliseconds();
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }


    public void run() {
        int step = 0;
        List<Command> steps = new ArrayList<>();
        if (robot.rangeSensor.getDistance(DistanceUnit.INCH) < 40) {
            if (prop == 1) {
                steps.add(new StrafeForDistance(robot, deadReckonDistance1, 2, 2, timer, 5, powerLevel, 1));
            } else if (prop == 2) {
                steps.add(new StrafeForDistance(robot, deadReckonDistance2, 2, 2, timer, 5, powerLevel, 1));
            } else {
                steps.add(new StrafeForDistance(robot, deadReckonDistance3, 2, 2, timer, 5, powerLevel, 1));
            }
            steps.add(new SlideToPosition(robot, timer, slidePosition, sPowerLevel, 3));
        } else {
            if (prop == 1) {
                steps.add(new RepositionAndSlide(robot, timer, RepositionDistance1, 5, powerLevel, slidePosition, sPowerLevel));
            } else if (prop == 2){
                steps.add(new RepositionAndSlide(robot, timer, RepositionDistance2, 5, powerLevel, slidePosition, sPowerLevel));
            } else {
                steps.add(new RepositionAndSlide(robot, timer, RepositionDistance3, 5, powerLevel, slidePosition, sPowerLevel));
            }
        }

        Command currentStep = steps.get(step);
        while(getState() == RUNNING) {
            // Execute current command
            if (currentStep.getState() == Command.STARTING) {
                currentStep.init();
            } else if (currentStep.getState() == Command.RUNNING) {
                currentStep.run();
            } else if (currentStep.getState() == Command.ENDING) {
                currentStep.end();
            } else if (currentStep.getState() == Command.DONE) {
                step++;
                if (step >= steps.size()) {
                    setState(ENDING);
                } else {
                    currentStep = steps.get(step);
                }
            }
        }
    }

    public void end() {
        if (getState() == ENDING) {
        }
        setState(DONE);
    }

}
