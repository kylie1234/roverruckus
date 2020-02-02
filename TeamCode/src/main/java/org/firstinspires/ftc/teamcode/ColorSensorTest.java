package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by kyliestruth 10/27/18.
 */

@Disabled
@Autonomous(name = "Color Sensor Test", group = "TankDrive")
public class ColorSensorTest extends LinearOpMode {

    HardwareBeep robot = new HardwareBeep();

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
        robot.latch.setPower(0);

        waitForStart();

        robot.colorSensor.enableLed(true);

        robot.latch.setPower(-1);
        while (robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) != 3) {
            telemetry.addData("Color Number", robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.update();
        }

        robot.latch.setPower(0);
        telemetry.addData("Stopped Servo", "Servo is stopped");
        telemetry.update();
        sleep(2000);

        robot.latch.setPower(1);
        while (robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) != 10) {
            telemetry.addData("Color Number", robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.update();
        }
    }
}
