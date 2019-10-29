package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TeleOp extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor extusion;

    Extrusion ex;

    @Override
    public void init() {
        ex = new Extrusion();
        ex.init(extusion, gamepad2);
    }

    @Override
    public void loop() {



    }
}
