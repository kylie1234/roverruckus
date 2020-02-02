package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @author Beep Patrol
 * <p>
 * <b>Summary:</b>
 * <p>
 * This is our main teleOp program which controls the robot during the driver controlled period.
 */
@TeleOp(name = "TeleOp Program", group = "TankDrive")
public class TeleOpProgram extends OpMode {

    // Declaring timers for the arm and basket.
    private ElapsedTime armtime = new ElapsedTime();
    private ElapsedTime baskettime = new ElapsedTime();
    // Setting arm_state, arm_extrusion_state, and basket_state values to zero for arm state machine.
    private int arm_state = 0;
    private int arm_extrusion_state = 0;
    private int basket_state = 0;
    // Calling hardware map.
    private HardwareBeep robot = new HardwareBeep();
    // Setting value to track whether the Y and A buttons are pressed to zero which is not pressed.
    private int buttonYPressed = 0;
    private int buttonAPressed = 0;
    // Setting initial direction to forward.
    private int direction = -1;
    // Setting scaling to full speed.
    private double scaleFactor = 1;
    private double scaleTurningSpeed = 1;

    /**
     * This method reverses the direction of the mecanum drive.
     */
    private void reverseDirection() {
        if (direction == 1) {
            direction = -1;
        } else if (direction == -1) {
            direction = 1;
        }
    }

    /**
     * This method scales the speed of the robot to .5.
     */
    private void scaleFactor() {
        if (scaleFactor == 0.5) {
            scaleFactor = 1;
        } else if (scaleFactor == 1) {
            scaleFactor = 0.5;
        }
    }

    /**
     * This method initializes hardware map.
     */
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
    }

    /**
     * This method sets motor power to zero
     */
    public void init_loop() {
        robot.lift.setPower(0);
        robot.latch.setPower(0);
        robot.intake.setPower(0);
        robot.arm.setPower(0);
        robot.armExtrusion.setPower(0);

    }

    /**
     * This method is the main body of our code which contains the code for each of the features on our robot used in teleOp
     */
    public void loop() {

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        // When the direction value is reversed this if statement inverts the addition and subtraction for turning.
        // Default mode: The robot starts with the scaleTurningSpeed set to 1, scaleFactor set to 1, and direction set to forward.
        if (direction == -1) {
            final double v1 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v2 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v3 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v4 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;

            robot.leftFront.setPower(v1);
            robot.rightFront.setPower(v2);
            robot.leftBack.setPower(v3);
            robot.rightBack.setPower(v4);

        } else {
            final double v1 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v2 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v3 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v4 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;

            robot.leftFront.setPower(v1);
            robot.rightFront.setPower(v2);
            robot.leftBack.setPower(v3);
            robot.rightBack.setPower(v4);
        }

        // When the y button has been pressed and released the direction is reversed.
        switch (buttonYPressed) {
            case (0):
                if (gamepad1.y) {
                    buttonYPressed = 1;
                }
                break;
            case (1):
                if (!gamepad1.y) {
                    reverseDirection();
                    buttonYPressed = 0;
                }
                break;
        }

        // When the a button has been pressed and released the speed is scaled to .5 power.
        switch (buttonAPressed) {
            case (0):
                if (gamepad1.a) {
                    buttonAPressed = 1;
                }
                break;
            case (1):
                if (!gamepad1.a) {
                    buttonAPressed = 0;
                    scaleFactor();
                }
                break;
        }

        // When the button below the right joystick is pressed JUST the turning speed power is set to .5.
        if (gamepad1.right_stick_button) {
            scaleTurningSpeed = 0.5;
        } else {
            scaleTurningSpeed = 1;
        }

        // When the dpad down is pressed down and the dpad up is not being pressed the lift power is set to 1.
        // When the dpad up is pressed down and the dpad down is not being pressed the lift power is set to -1.
        // Otherwise the power is set to zero
        if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            robot.lift.setPower(1);
        } else if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.lift.setPower(-1);
        } else {
            robot.lift.setPower(0);
        }

        // If the right dpad is being pressed the latch power is set to -1.
        // If the left dpad is being pressed the latch power is set to 1.
        // Otherwise the power is set to zero.
        if (gamepad2.dpad_right) {
            robot.latch.setPower(-1);
        } else if (gamepad2.dpad_left) {
            robot.latch.setPower(1);
        } else robot.latch.setPower(0);

        // When the right trigger is being pressed the intake power is set to -.5.
        // When the right bumper is being pressed the intake power is set to .5.
        // Otherwise the power is set to zero.
        if (gamepad1.right_trigger > 0) {
            robot.intake.setPower(-0.75);
        } else if (gamepad1.right_bumper) {
            robot.intake.setPower(0.75);
        } else {
            robot.intake.setPower(0);
        }

        switch (arm_extrusion_state) {
            case 0:
                // when the right bumper is being pressed and the touch sensor is not being set the armExtrusion power is set to 1 and the basket position is set to .5.
                // Once those conditions are met the state advances to the next case.
                // When the right trigger is being pressed the armExtrusion power is set to -1 and the basket position is set to .4.
                // Otherwise the power is set to 0.
                if (gamepad2.right_bumper && robot.touchSensor.getState()) {
                    robot.armExtrusion.setPower(1);
                    robot.basket.setPosition(.5);
                    arm_extrusion_state++;

                } else if (gamepad2.right_trigger > 0) {
                    robot.armExtrusion.setPower(-1);
                    robot.basket.setPosition(.4);
                } else {
                    robot.armExtrusion.setPower(0);
                }
                break;
            case 1:
                // Once the right bumper is not being pressed it advances to the next state.
                if (!gamepad2.right_bumper) {
                    arm_extrusion_state++;
                }
                break;
            case 2:
                // Last state before it goes back to state 0.
                // When the touch sensor is pressed it advances to state zero.
                // If the right bumper is pressed the state advances to the next case.
                if (!robot.touchSensor.getState()) {
                    robot.armExtrusion.setPower(0);
                    arm_extrusion_state = 0;
                } else if (gamepad2.right_bumper) {
                    arm_extrusion_state++;
                }
                break;
            case 3:
                // This case sets the power to zero once the right bumper has been released and returns to state zero.
                // This allows the drivers to terminate the movement of the arm to avoid damage to the robot.
                if (!gamepad2.right_bumper) {
                    robot.armExtrusion.setPower(0);
                    arm_extrusion_state = 0;
                }
                break;
        }

        // In this state machine: 0 = waiting for command, 1 = commanded, 2 = waiting for the arm to not be busy
        switch (arm_state) {
            case 0:
                //  This case is the constant state that waits for the trigger or bumper to be pressed.

                // The arm is set to default to the game pad 2 right stick
                robot.arm.setPower(gamepad2.right_stick_y * -.75);

                // When the left bumper on game pad 1 is pressed the arm moves to a vertical set position of about 90 degrees.
                // When the left trigger on game pad 1 is pressed the arms moves to a horizontal set position of about 180 degrees.
                if (gamepad1.left_bumper) {
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(500);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(.75);
                    arm_state = 1;

                } else if (gamepad1.left_trigger > 0) {
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-580);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    arm_state = 1;

                    //  Once it recognizes that the controller has been moved, and the power is set, then it advances to state 1.
                }
                break;
            case 1:
                // Resetting the timer for the arm
                armtime.reset();
                // If the arm is moving advance to state 2
                if (robot.arm.isBusy()) {
                    arm_state = 2; // moving
                }
                break;
            case 2:
                // When the arm is not busy and the timer has reached 2 it returns to state 0 and set the arm power to zero.
                if (!robot.arm.isBusy() || armtime.seconds() >= 2) {
                    arm_state = 0;
                    robot.arm.setPower(0);
                }
                break;

        }

        // Manual Override for the arm:
        // If the game pad 2 right joystick is between the values .05 and 1 or -1 and -.05 then the arm is set to run without encoders and it returns to state zero.
        if ((gamepad2.right_stick_y > .05 && gamepad2.right_stick_y <= 1) ||
                (gamepad2.right_stick_y < -.05 && gamepad2.right_stick_y >= -1)) {
            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm_state = 0;
        }

        // When the game pad 2 a button is pressed set the basket position to 0
        // When the game pad 2 x button is pressed set the basket position to .5.
        // When the game pad 2 b button is pressed set the basket position to .9.
        if (gamepad2.a) {
            robot.basket.setPosition(0);
            telemetry.addData("Button a pressed", gamepad2.a);
            telemetry.update();
        } else if (gamepad2.x) {
            robot.basket.setPosition(.5);
            telemetry.addData("Button x pressed", gamepad2.x);
            telemetry.update();
        } else if (gamepad2.b) {
            robot.basket.setPosition(.9);
        }

        switch (basket_state) {
            case (0):
                // When the game pad 2 y button is pressed this state resets the timer, sets the position to .25 and advances to the next state.
                if (gamepad2.y) {
                    baskettime.reset();
                    robot.basket.setPosition(.25);
                    basket_state++;
                }
                break;
            case (1):
                // Once the basket reaches the .25 position and the timer has reached .25 seconds the robot basket position is set to zero and it returns to state zero.
                if (robot.basket.getPosition() >= .25 && baskettime.seconds() > .25) {
                    robot.basket.setPosition(0);
                    basket_state = 0;
                }
                break;
        }

        // Telemetry
        telemetry.addData("y Button", buttonYPressed);
        telemetry.addData("Lift Encoder Ticks", robot.lift.getCurrentPosition());
        telemetry.addData("Color Number", robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
        telemetry.addData("arm_state", arm_state);
        telemetry.addData("arm_extrusion_state", arm_extrusion_state);
        telemetry.addData("touch sensor", robot.touchSensor.getState());
        telemetry.addData("Scale Factor", scaleFactor);
        telemetry.addData("Direction", direction);
        telemetry.addData("left front power", robot.leftFront.getPower());
        telemetry.addData("left back power", robot.leftBack.getPower());
        telemetry.addData("right front power", robot.rightFront.getPower());
        telemetry.addData("right back power", robot.rightBack.getPower());
        telemetry.addData("Arm Encoder Ticks", robot.arm.getCurrentPosition());
        telemetry.addData("Arm Power", robot.arm.getPower());
        telemetry.addData("Arm Extrusion Encoder Ticks", robot.armExtrusion.getCurrentPosition());
        telemetry.addData("Arm Extrusion Power", robot.armExtrusion.getPower());
        telemetry.update();
    }

    /**
     * This method sets the buttons to not being pressed, sets the motor power to zero, and terminates the program.
     */
    public void stop() {

        buttonYPressed = 0;
        buttonAPressed = 0;
        robot.lift.setPower(0);
        robot.latch.setPower(0);
        robot.intake.setPower(0);
        robot.arm.setPower(0);
        robot.armExtrusion.setPower(0);

    }
}