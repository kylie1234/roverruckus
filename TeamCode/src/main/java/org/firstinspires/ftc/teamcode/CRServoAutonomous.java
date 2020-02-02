package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "CRServo Autonomous", group = "Beep")
public class CRServoAutonomous extends LinearOpMode {

    HardwareMapTestingCRServo robot = new HardwareMapTestingCRServo();

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();
        robot.init(hardwareMap);
        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();


        /**
         Wait for start button.
         */

        robot.basket.setPosition(0.46);

        waitForStart();


        /**
         landing our robot
         */

//        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.lift.setTargetPosition(-12500);
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lift.setPower(1);
//
//        while (opModeIsActive() &&
//                robot.lift.isBusy()){
//            telemetry.addData("Lift Encoder Ticks", robot.lift.getCurrentPosition());
//            telemetry.update();
//
//        }
//
//        robot.lift.setPower(0);
        robot.basket.setPosition(.46);
        robot.basket.setPosition(.50);
        sleep(1000);
    }
}