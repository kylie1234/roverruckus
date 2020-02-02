package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by kyliestruth 10/5/17.
 */

@Disabled
@TeleOp(name = "Auto Latch With Override", group = "TankDrive")
public class AutoLatchWithOverride extends OpMode {

    public ElapsedTime autolatchtime = new ElapsedTime();
    public ElapsedTime colorsensortime = new ElapsedTime();
    public ElapsedTime colorSensorTimeOutOpen = new ElapsedTime();
    public ElapsedTime colorSensorTimeOutClose = new ElapsedTime();
    int auto_latch_open = 0;
    int auto_latch_close = 0;
    private HardwareBeep robot = new HardwareBeep();
    private boolean manual_mode = false;
    private boolean latch_open_mode = false;
    private boolean latch_close_mode = false;

    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
    }

    public void loop() {

        if (!manual_mode && !latch_open_mode) {
            if (auto_latch_open == 2 && colorSensorTimeOutOpen.seconds() > 1.15) {
                robot.latch.setPower(0);
                auto_latch_open = 0;
            }
            switch (auto_latch_open) {
                case 0:
                    if (gamepad2.dpad_right) {
                        autolatchtime.reset();
                        auto_latch_open++;
                    }
                    break;
                case 1:
                    colorSensorTimeOutOpen.reset();
                    robot.latch.setPower(-1);
                    auto_latch_open++;
                    break;
                case 2:
                    if (colorsensortime.milliseconds() > 200) {
                        if (robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 3) {
                            robot.latch.setPower(0);
                            auto_latch_open = 0;
                        }
                        colorsensortime.reset();
                    }
                    break;
            }
        }
        if (!manual_mode && !latch_close_mode) {
            if (auto_latch_close == 2 && colorSensorTimeOutClose.seconds() > 1.15) {
                robot.latch.setPower(0);
                auto_latch_close = 0;
            }
            switch (auto_latch_close) {
                case 0:
                    if (gamepad2.dpad_left) {
                        autolatchtime.reset();
                        auto_latch_close++;
                    }
                    break;
                case 1:
                    colorSensorTimeOutClose.reset();
                    robot.latch.setPower(1);
                    auto_latch_close++;
                    break;
                case 2:
                    if (colorsensortime.milliseconds() > 200) {
                        if (robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 10) {
                            robot.latch.setPower(0);
                            auto_latch_close = 0;
                        }
                        colorsensortime.reset();
                    }
                    break;
            }
        }


        if (autolatchtime.seconds() > 1 && (gamepad2.dpad_right || gamepad2.dpad_left)) {
            manual_mode = true;
            robot.latch.setPower(0);
        }

        if (manual_mode) {
            if (gamepad2.dpad_right) {
                robot.latch.setPower(-1);
            } else if (gamepad2.dpad_left) {
                robot.latch.setPower(1);
            } else robot.latch.setPower(0);
        }

    }

    public void stop() {

    }
}