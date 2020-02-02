package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "GyroDriveTesting", group = "Exercises")
public class GyroDriveTesting extends LinearOpMode {

    public HardwareBeep robot = new HardwareBeep();
    public LibraryGyro gyro = new LibraryGyro();
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();

        robot.init(hardwareMap);
        gyro.init(robot, telemetry);
        gyroDrive.init(robot, telemetry, robot.rightBack);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyroTurn is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        //gyroDrive.gyroDrive(.3, 10000, 0);
        gyroDrive.gyroDriveVariableP(.5, 2000, 0, .01);
        sleep(2000);
        gyroDrive.gyroDriveVariableP(.5, 2000, 0, .02);
        sleep(2000);
        gyroDrive.gyroDriveVariableP(.5, 2000, 0, .05);
        sleep(2000);
        gyroDrive.gyroDriveVariableP(.5, 2000, 0, .1);
        sleep(2000);
        gyroDrive.gyroDriveVariableP(.7, 2000, 0, .01);
        sleep(2000);
        gyroDrive.gyroDriveVariableP(.7, 2000, 0, .02);
        sleep(2000);
        gyroDrive.gyroDriveVariableP(.7, 2000, 0, .05);
        sleep(2000);
        gyroDrive.gyroDriveVariableP(.7, 2000, 0, .1);
        sleep(2000);

    }
}