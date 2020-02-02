package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by kyliestruth 10/27/18.
 */

@Disabled
@Autonomous(name = "Auto Drive by Gyro", group = "TankDrive")
public class AutoDriveByGyro extends LinearOpMode {
    HardwareBeep robot = new HardwareBeep();
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        gyroDrive.init(robot, telemetry, robot.leftBack);
        telemetry.addData("Say", "Hello Driver");

        waitForStart();

        gyroDrive.gyroDrive(.5, 4000, 0);

    }
}
