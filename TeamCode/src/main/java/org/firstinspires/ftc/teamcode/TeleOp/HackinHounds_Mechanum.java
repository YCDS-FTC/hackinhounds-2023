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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;



/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Mechanum", group="Linear OpMode")

public class HackinHounds_Mechanum extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private HackinHoundsHardware robot = new HackinHoundsHardware();
    double shift = 1;
    boolean slideMoving = false;
    boolean wristMoving = false;
    boolean hold = true;
    int currentPos = 0;

    //PID Stuff
    double integralSum = 0;
    double Kp = 10;
    double Ki = 0;
    double Kd = 0;
    double lasterror = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            if (gamepad1.y) {
                shift = 1;
            }
            if (gamepad1.x) {
                shift = 0.75;
            }
            if (gamepad1.a) {
                shift = 0.5;
            }
            if (gamepad1.b) {
                shift = 0.25;
            }
            double facing = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double rotX = x * Math.cos(-facing) - y * Math.sin(-facing);
            rotX = rotX * 1.1;
            double rotY = x * Math.sin(-facing) + y * Math.cos(-facing);

            double d = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);

            double lf = (rotY + rotX + rx) / d;
            double lb = (rotY - rotX + rx) / d;
            double rf = (rotY - rotX - rx) / d;
            double rb = (rotY + rotX - rx) / d;

            robot.leftFront.setVelocity(2000 * lf * shift);
            robot.leftBack.setVelocity(2000 * lb * shift);
            robot.rightFront.setVelocity(2000 * rf * shift);
            robot.rightBack.setVelocity(2000 * rb * shift);



//            double slidePower = -gamepad2.left_stick_y;
//            int slideCurrentPos = robot.slide.getCurrentPosition();
//            if ((slidePower > 0.3) && (slideCurrentPos < 1100)) {
//                robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                slideMoving = true;
//                robot.slide.setPower(slidePower);
//            } else if ((slidePower < -0.3) && (slideCurrentPos > -1000)) {
//                robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                slideMoving = true;
//                robot.slide.setPower(slidePower);
//            } else {
//                if (slideMoving) {
//                    slideMoving = false;
//                    robot.slide.setPower(0);
//                }
//            }

            double slidePower = -gamepad2.left_stick_y;
            int slideCurrentPos = robot.slide.getCurrentPosition();
            if ((slidePower > 0.1) && (slideCurrentPos < 8000)) {
                robot.slide.setPower(slidePower);
            } else if ((slidePower < -0.1) && (slideCurrentPos > 0)) {
                robot.slide.setPower(slidePower);
            } else {
                if (!slideMoving) {
                    robot.slide.setPower(0);
                }
            }


            if (gamepad2.dpad_up) {
                slideMoving = true;
            }

            if (slideMoving) {
                robot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.wrist.setPosition(0.5);
                double power = PIDControl(1000, robot.slide.getVelocity());
                robot.slide.setPower(power);
            }


            double wristPower = gamepad2.right_stick_y * 0.01;
            robot.wrist.setPosition(clamp(robot.wrist.getPosition() + wristPower, 0.5, 0.8));

            if (gamepad2.right_trigger >= 0.1) {
                robot.top_claw.setPosition(0.75);
            }
            if (gamepad2.right_bumper) {
                robot.top_claw.setPosition(0.5);
            }
            if (gamepad2.left_trigger >= 0.1) {
                robot.bottom_claw.setPosition(0.15);
            }
            if (gamepad2.left_bumper) {
                robot.bottom_claw.setPosition(0.5);
            }

            if (gamepad1.back) {
                robot.imu.resetYaw();
            }

            if (hold == true) {
                robot.launcher.setPosition(0.7);
                robot.wrist.setPosition(0.5);
                hold = false;
            }

            if (gamepad2.back) {
                robot.launcher.setPosition(0.3);
            }

            //Telemetry
            telemetry.addData("Left stick:", "%f", gamepad2.left_stick_y);
            telemetry.addData("slide Pos:", "%d", robot.slide.getCurrentPosition());
            telemetry.addData("slide Volocity:", "%f", robot.slide.getVelocity());
            telemetry.addData("Wrist Pos:", "%f", robot.wrist.getPosition());
            telemetry.addData("Launcher Pos:", "%f", robot.launcher.getPosition());
            telemetry.addData("LF", "%d", robot.leftBack.getCurrentPosition());
            telemetry.addData("Angle:", "%f", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
    private double clamp(double x, double min, double max) {

        return Math.max(min,Math.min(max,x));
    }
    private double PIDControl (double reference, double state) {
        double error = reference - state;
        integralSum += error * runtime.seconds();
        double derivitive = (error - lasterror) / runtime.seconds();
        lasterror = error;
        runtime.reset();

        double output = (error * Kp) + (derivitive * Kd) + (integralSum * Ki);
        return output;
    }
}
