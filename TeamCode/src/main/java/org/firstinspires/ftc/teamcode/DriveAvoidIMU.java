package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
@Disabled
@TeleOp(name = "Drive Avoid IMU", group = "Exercises")
public class DriveAvoidIMU extends LinearOpMode {

    DigitalChannel touch;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .05, correction;
    boolean aButton, bButton;
    HardwareBeep robot = new HardwareBeep();   // Use a Pushbot's hardware

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        robot.leftFront = hardwareMap.dcMotor.get("left_front");
        robot.rightFront = hardwareMap.dcMotor.get("right_front");
        robot.leftBack = hardwareMap.dcMotor.get("left_back");
        robot.rightBack = hardwareMap.dcMotor.get("right_back");

//        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
//        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // get a reference to REV Touch sensor.
        // touch = hardwareMap.digitalChannel.get("touch_sensor");

        robot.init(hardwareMap);
        int i = 0;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating mode...");

        // make sure the imu gyroTurn is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());

        // wait for start button.
        waitForStart();

        telemetry.addData("Mode", "running");

        sleep(500);

        // drive until end of period.

        while (opModeIsActive()) {

            try {
                // Use gyroTurn to drive in a straight line.
                correction = checkDirection();

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 power", power);

                //setMotorPower(-power + correction, -power);
                telemetry.addData("power", -power);
                telemetry.addData("setting left/right/front/back drives", "done " + i++);

                // We record the sensor values because we will test them in more than
                // one place with time passing between those places. See the lesson on
                // Timing Considerations to know why.

                aButton = gamepad1.a;
                telemetry.addData("gamepad1a", gamepad1.toString());
                telemetry.addData("aButton", aButton);

                bButton = gamepad1.b;
                telemetry.addData("gamepad1a", gamepad1.toString());
                telemetry.addData("bButton", bButton);

//
                if (!aButton || !bButton) { //TODO: remove NOT condition ! when using working gamepad
                    setMotorPower(power, power);
                    telemetry.addData("power", power);
                    telemetry.addData("aButton", aButton);

                    sleep(50);

                    // turn 90 degrees right.
                    if (aButton) rotate(-90, power);

                    // turn 90 degrees left.
                    if (bButton) rotate(90, power);
                } else {
                    // stop.
                    setMotorPower(0, 0);
                }

            } catch (Exception e) {
                telemetry.addData("Exception: ", e);
                telemetry.update();
                throw e;
            }
            telemetry.update();

        }
        // turn the motors off.
        setMotorPower(0, 0);

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {

        double leftFrontPower, rightFrontPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftFrontPower = -power;
            rightFrontPower = power;
        } else if (degrees > 0) {   // turn left.
            leftFrontPower = power;
            rightFrontPower = -power;
        } else return;

        // set power to rotate.
        setMotorPower(leftFrontPower, rightFrontPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
            }

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private void setMotorPower(double powerL, double powerR) {

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftBack.setPower(powerL);
        robot.leftFront.setPower(powerL);
        robot.rightBack.setPower(powerR);
        robot.rightFront.setPower(powerR);
    }
}