package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Disabled
@Autonomous(name = "Michael Gryo Testing 2", group = "Exercises")
public class MichaelGyroTurnTest2 extends LinearOpMode {

    HardwareBeep robot = new HardwareBeep();
    DcMotor LeftFront;
    DcMotor LeftBack;
    DcMotor RightFront;
    DcMotor RightBack;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    double angle_variable;
    boolean aButton, bButton, touched;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        LeftFront = hardwareMap.dcMotor.get("left_front");
        LeftBack = hardwareMap.dcMotor.get("left_back");

        RightFront = hardwareMap.dcMotor.get("right_front");
        RightBack = hardwareMap.dcMotor.get("right_back");


        // get a reference to REV Touch sensor.
//        touch = hardwareMap.digitalChannel.get("touch_sensor");

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

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyroTurn is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        GyroTurn(90);
        while (opModeIsActive()) {

            getAngle();
            angle_variable = getAngle();
            telemetry.addData("Mode", "running");
            telemetry.addData("Current Angle", getAngle());
            telemetry.addData("Current Angle", angle_variable);
            telemetry.update();

            // drive until end of period.
        }
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
    private void GyroTurn(int degrees) {

        resetAngle();

        double realign;

        if (degrees < 0) {
            //rotates left
            while (opModeIsActive() && getAngle() > degrees) {

                double Proportional = (degrees - getAngle()) / degrees;
                double speed = Math.round(Proportional);

                telemetry.addData("Telemetry 1", "Active");
                LeftFront.setPower(-speed);
                LeftBack.setPower(-speed);
                RightFront.setPower(speed);
                RightBack.setPower(speed);
                // set leftPower to -speed;
                // set rightPower to speed;
                telemetry.addData("Telemetry 2", "Active");


            }

            telemetry.addData("Telemetry 3", "Active");

            LeftFront.setPower(0);
            LeftBack.setPower(0);
            RightFront.setPower(0);
            RightBack.setPower(0);
            // set leftPower to 0;
            // set rightPower to 0;

            realign = degrees - getAngle();

            while (opModeIsActive() && realign > 0) {

                LeftFront.setPower(.1);
                LeftBack.setPower(.1);
                RightFront.setPower(-.1);
                RightBack.setPower(-.1);
                // set leftPower to .1
                // set rightPower to -.1
            }
        } else if (degrees > 0) {
            //rotates right

            while (opModeIsActive() && getAngle() < degrees) {

                double Proportional = (degrees - getAngle()) / degrees;
                double speed = Math.round(Proportional);

                telemetry.addData("Telemetry 4", "Active");

                LeftFront.setPower(speed);
                LeftBack.setPower(speed);
                RightFront.setPower(speed);
                RightBack.setPower(speed);
                // set leftPower to speed;
                // set rightPower to -speed;
                telemetry.addData("Telemetry 5", "Active");


            }

            telemetry.addData("Telemetry 6", "Active");

            LeftFront.setPower(0);
            LeftBack.setPower(0);
            RightFront.setPower(0);
            RightBack.setPower(0);
            // set leftPower to 0;
            // set rightPower to 0;

            realign = degrees - getAngle();

            while (opModeIsActive() && realign > 0) {

                LeftFront.setPower(-.1);
                LeftBack.setPower(-.1);
                RightFront.setPower(.1);
                RightBack.setPower(.1);
                // set leftPower to -.1
                // set rightPower to .1

            }
        } else return;


        resetAngle();


    }
}