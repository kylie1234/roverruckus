package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareMapTestingArm {


    public DcMotor arm = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public HardwareMapTestingArm() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;


        arm = hwMap.get(DcMotor.class, "basket");

        arm.setPower(0);


    }
}
