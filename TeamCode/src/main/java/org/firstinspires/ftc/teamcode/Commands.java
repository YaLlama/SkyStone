package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Commands {
    public static void initialize(){
        DcMotor motorLeftFront = null;
        DcMotor motorRightFront = null;
        DcMotor motorRightBack = null;
        DcMotor motorLeftBack = null;

        motorLeftBack = hardwareMap.dcMotor.get("motorLeftFront");

    }
}
