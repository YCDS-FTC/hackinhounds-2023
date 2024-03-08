/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autonomous_NewRoutines;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutonCommands.MoveAndSlide;
import org.firstinspires.ftc.teamcode.AutonCommands.MoveForDistance;
import org.firstinspires.ftc.teamcode.AutonCommands.Reposition;
import org.firstinspires.ftc.teamcode.AutonCommands.RepositionAndSlide;
import org.firstinspires.ftc.teamcode.AutonCommands.RepositionWithSlowDown;
import org.firstinspires.ftc.teamcode.AutonCommands.SetClaws;
import org.firstinspires.ftc.teamcode.AutonCommands.SetWrist;
import org.firstinspires.ftc.teamcode.AutonCommands.StrafeForDistance;
import org.firstinspires.ftc.teamcode.AutonCommands.TestForOthers;
import org.firstinspires.ftc.teamcode.AutonCommands.ToRed;
import org.firstinspires.ftc.teamcode.AutonCommands.ToRedStrafe;
import org.firstinspires.ftc.teamcode.AutonCommands.TurnToHeading;
import org.firstinspires.ftc.teamcode.AutonCommands.WaitForTime;
import org.firstinspires.ftc.teamcode.Hardware.Command;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;

import java.util.ArrayList;
import java.util.List;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *A
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "S-RedRight-TFBF")
public class S_Red_Right_TFBF extends LinearOpMode {
    private HackinHoundsHardware robot = new HackinHoundsHardware();
    //Create elapsed time variable and an instance of elapsed time
    private ElapsedTime runtime = new ElapsedTime();
    private boolean done = false;
    private int propPos = 0;

    @Override
    public void runOpMode() {
        double startTime = 0;
        robot.init(hardwareMap);
        robot.imu.resetYaw();
        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        List<Command> steps = new ArrayList<>();

        int step = 0;
        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        // Waiting for start
        while(!isStarted()) {
            HuskyLens.Block[] blocks = robot.huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                //int thisColorID = blocks[i].id;
                //telemetry.addData("This Color ID", thisColorID);
                telemetry.addData("Block", blocks[i].toString());
            }
            telemetry.addLine("On the right");
            for (int i = 0; i < blocks.length; i++) {
                if (blocks[i].id == 1 && blocks[i].y > 100) {
                    if (blocks[i].x > 160 ) {
                        propPos = 2;
                        telemetry.addLine("On the Middle");
                    } else {
                        propPos = 1;
                        telemetry.addLine("On the Left");
                    }
                }
            }
            telemetry.update();
        }
        startTime = runtime.seconds();
        HuskyLens.Block[] blocks = robot.huskyLens.blocks();
        propPos = 3;
        telemetry.addLine("On the right");
        for (int i = 0; i < blocks.length; i++) {
            if (blocks[i].id == 1 && blocks[i].y > 100) {
                if (blocks[i].x > 160 ) {
                    propPos = 2;
                    telemetry.addLine("On the Middle");
                } else {
                    propPos = 1;
                    telemetry.addLine("On the Left");
                }
            }
        }
        telemetry.update();
        robot.launcher.setPosition(robot.launcherUp);

        steps.add(new SetClaws(robot, runtime, 0.01, "top", robot.topClawClose));
        steps.add(new SetClaws(robot, runtime, 1, "bottom", robot.bottomClawClose));

        steps.add(new SetWrist(robot, runtime, 0.1, robot.wristUp));

        steps.add(new MoveForDistance(robot, 2, 0, 0, runtime, 3, 0.5, 1));

        steps.add(new RepositionWithSlowDown(robot, runtime, 20, 5, 0.75));

        steps.add(new TurnToHeading(robot, runtime, -90, 0.5, 5));

        if (propPos == 3) {
            steps.add(new RepositionAndSlide(robot, runtime, 20, 5, 0.3, -1200, 1));
        } else if (propPos == 2) {
            steps.add(new RepositionAndSlide(robot, runtime, 25, 5, 0.3, -1200, 1));
        } else {
            steps.add(new RepositionAndSlide(robot, runtime, 33, 5, 0.3, -1200, 1));
        }

        steps.add(new TurnToHeading(robot, runtime, -90, 0.6, 2));

        steps.add(new MoveForDistance(robot, 5, 1, 1, runtime, 5, 0.5, 1));

        steps.add(new WaitForTime(robot, runtime, 0.2));

        steps.add(new SetClaws(robot, runtime, 0.2, "bottom", robot.bottomClawOpen));

        steps.add(new MoveAndSlide(robot, 20, 2, 2, runtime, 3, -0.5, 0, 1));

        steps.add(new Reposition(robot, runtime, 28, 5, 0.3));

        steps.add(new SetWrist(robot, runtime, 0.01, robot.wristDown));

        if (propPos == 3) {
            steps.add(new ToRed(robot, runtime, 5, -0.5));
            steps.add(new MoveForDistance(robot, 1, 0, 0, runtime, 5, 0.3, 1));
            steps.add(new WaitForTime(robot, runtime, 0.2));
            steps.add(new SetClaws(robot, runtime, 0.5, "top", robot.topClawOpen));
            steps.add(new SetWrist(robot, runtime, 0.1, robot.wristUp));
            steps.add(new StrafeForDistance(robot, 23, 7, 7, runtime, 5, -0.6, 1));
            steps.add(new TurnToHeading(robot, runtime, 90, 0.5, 3));
            steps.add(new MoveForDistance(robot, 24, 10, 10, runtime, 5, 0.75, true, -90, 0.03));
            steps.add(new MoveAndSlide(robot, 40, 10, 10, runtime, 5, 0.75, -470, 1, true, -90, 0.03));
        } else if (propPos == 2) {
            steps.add(new MoveForDistance(robot, 8, 5, 5, runtime, 5, -0.5, 1));
            steps.add(new TurnToHeading(robot, runtime, 90, 0.5, 3));
            steps.add(new ToRedStrafe(robot, runtime, 5, 0.3));
            steps.add(new StrafeForDistance(robot, 1, 0, 1, runtime, 1,-0.3,1));
            steps.add(new WaitForTime(robot, runtime, 0.2));
            steps.add(new SetClaws(robot, runtime, 0.5, "top", robot.topClawOpen));
            steps.add(new SetWrist(robot, runtime, 0.01, robot.wristUp));
            steps.add(new TurnToHeading(robot, runtime, 90, 0.5, 1));
            steps.add(new StrafeForDistance(robot, 14, 7, 7, runtime, 5, 0.6, 1));
            steps.add(new MoveForDistance(robot, 20, 10, 10, runtime, 5, 0.75, true, -90, 0.03));
            steps.add(new MoveAndSlide(robot, 40, 10, 10, runtime, 5, 0.75, -470, 1, true, -90, 0.03));
        } else if (propPos == 1) {
            steps.add(new MoveForDistance(robot, 15, 5, 5, runtime, 5, -0.5, 1));
            steps.add(new TurnToHeading(robot, runtime, 85, 0.5, 3));
            steps.add(new ToRed(robot, runtime, 5, 0.5));
            steps.add(new MoveForDistance(robot, 5, 1, 1, runtime, 5, -0.3, 1));
            steps.add(new WaitForTime(robot, runtime, 0.2));
            steps.add(new SetClaws(robot, runtime, 0.5, "top", robot.topClawOpen));
            steps.add(new MoveForDistance(robot, 5, 1, 1, runtime, 5, -0.3, 1));
            steps.add(new StrafeForDistance(robot, 25, 7, 7, runtime, 5, 0.6, 1));
            steps.add(new MoveForDistance(robot, 20, 10, 10, runtime, 5, 0.75, true, -90, 0.03));
            steps.add(new MoveAndSlide(robot, 40, 10, 10, runtime, 5, 0.75, -470, 1, true, -90, 0.03));
        }

        steps.add(new SetWrist(robot, runtime, 0.01, robot.wristDown));

        steps.add(new TurnToHeading(robot, runtime, 90, 0.7, 1));

        steps.add(new MoveForDistance(robot, 8, 2, 2, runtime, 5, 0.3, true, -90, 0.03));

        steps.add(new SetClaws(robot, runtime, 0.01, "top", robot.topClawClose));
        steps.add(new SetClaws(robot, runtime, 1, "bottom", robot.bottomClawClose));

        steps.add(new MoveForDistance(robot, 90, 10, 10, runtime, 5, -1, true, -90, 0.05));

        steps.add(new TurnToHeading(robot, runtime, 270, 0.5, 2));

        steps.add(new TestForOthers(robot, runtime, 30, 33, 30, 20, 14,20,5, -0.5, -2000, 1, propPos));

        //33, 20, 27
//        if (propPos == 1) {
//            steps.add(new RepositionAndSlide(robot, runtime, 30, 5, -0.3, -2000, 1));
//        } else if (propPos == 2) {
//            steps.add(new RepositionAndSlide(robot, runtime, 33, 5, -0.3, -2000, 1));
//        } else {
//            steps.add(new RepositionAndSlide(robot, runtime, 30, 5, -0.3, -2000, 1));
//        }

        steps.add(new SetWrist(robot, runtime, 0.01, robot.wristUp));

        steps.add(new TurnToHeading(robot, runtime, 270, 0.6, 2));

        steps.add(new ToRed(robot, runtime, 5, 0.5));

        steps.add(new MoveForDistance(robot, 4, 1, 1, runtime, 5, 0.5, 1));

        steps.add(new SetClaws(robot, runtime, 0.1, "bottom", robot.bottomClawOpen));
        steps.add(new SetClaws(robot, runtime, 0.5, "top", robot.topClawOpen));

        steps.add(new MoveForDistance(robot, 5, 2, 2, runtime, 3, -0.5, 1));


        // This is where we build the autonomous routine
        Command currentStep = steps.get(step);
        while(opModeIsActive() && !done) {
            // Execute current command
            if(currentStep.getState() == Command.STARTING) {
                telemetry.addData("Step:", "Starting");
                currentStep.init();
            } else if (currentStep.getState() == Command.RUNNING) {
                telemetry.addData("Step:", "Running");
                currentStep.run();
            } else if (currentStep.getState() == Command.ENDING) {
                telemetry.addData("Step:", "Ending");
                currentStep.end();
            } else if (currentStep.getState() == Command.DONE) {
                telemetry.addData("Step:", "Done");
                step++;
                if (step >= steps.size()){
                    done = true;
                } else {
                    currentStep = steps.get(step);
                }
            }
            telemetry.addData("slide Pos:", "%d", robot.slide.getCurrentPosition());
            telemetry.update();
        }
    }
}