package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareMapTestingCRServo {

    public Servo basket = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public HardwareMapTestingCRServo() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;


        basket = hwMap.get(Servo.class, "basket");

        basket.setPosition(0.46);


    }
}
