package org.firstinspires.ftc.teamcode.AA_AutonCommands;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AAmir_Auto")
public class AAmir_Auton extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo claw = null;
    private Servo claw2 = null;
    private HuskyLens husky_lens = null;
    private Rev2mDistanceSensor distance_sensor = null;

    private int BlockPosition = 0;

    @Override
    public void runOpMode(){
        leftDrive = hardwareMap.get(DcMotor.class,"left_drive");
        rightDrive = hardwareMap.get(DcMotor.class,"right_drive");
        claw = hardwareMap.get(Servo.class,"claw");
        husky_lens = hardwareMap.get(HuskyLens.class,"husky_lens");
        distance_sensor = hardwareMap.get(Rev2mDistanceSensor.class,"distance_sensor");
        claw2 = hardwareMap.get(Servo.class, "claw2");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        husky_lens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        while(opModeIsActive()){
            if (runtime.seconds() < (.5)) {

            } else if (runtime.seconds() < 3) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                HuskyLens.Block[] blocks = husky_lens.blocks();
                telemetry.addData("Block count", blocks.length);
                BlockPosition = 3;
                for (int i = 0; i < blocks.length; i++) {
                    if (blocks[i].id == 1 && blocks[i].x < 110) {
                        BlockPosition = 1;
                    }
                    else if (blocks[i].id == 1 && blocks[i].x >= 110) {
                        BlockPosition = 2;
                    }

                }
            } else if (runtime.seconds() < 5) {
                if (BlockPosition == 1) {
                    if (runtime.seconds()<4) {
                        leftDrive.setPower(0.45);
                        rightDrive.setPower(0.45);
                    } else {
                        leftDrive.setPower(-0.25);
                        rightDrive.setPower(0.25);
                    }

                } else if (BlockPosition == 2) {
                    while (distance_sensor.getDistance(DistanceUnit.INCH) > 3.75){
                        leftDrive.setPower(0.5);
                        rightDrive.setPower(0.5);
                    }
                     if(distance_sensor.getDistance(DistanceUnit.INCH) < 3.75){
                        rightDrive.setPower(0);
                        leftDrive.setPower(0);
                        claw.setPosition(1);
                        claw2.setPosition(0 );
                    }
                            }



                } else if (BlockPosition == 3) {
                    leftDrive.setPower(0.45);
                    rightDrive.setPower(0.45);
                    claw.setPosition(1);
                    claw2.setPosition(0);

                }
            }

            telemetry.addData("BlockPosition","%d",BlockPosition);
            telemetry.addData("range", String.format("%.01f in",distance_sensor.getDistance(DistanceUnit.INCH)));
            telemetry.update();





            }
        }


