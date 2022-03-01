package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 1ssem0 on 2019/5/7.
 */

public class srsServoTest {
    public static HardwareMap hwmap;
    public static Servo srs = null;

    public void robotInit(HardwareMap hardwareMap){
        this.hwmap = hardwareMap;
        this.srs = hwmap.servo.get("srs");
    }
}
