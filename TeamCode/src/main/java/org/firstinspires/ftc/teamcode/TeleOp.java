package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleOp extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor extusion;

    Servo clampServo;
    Servo rotationServo;

    Extrusion ex;

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        extusion = hardwareMap.dcMotor.get("extrusion");

        clampServo = hardwareMap.servo.get("clamp");
        rotationServo = hardwareMap.servo.get("rotation");

        ex = new Extrusion(extusion, gamepad1, gamepad2, leftFront, leftBack, rightBack, rightFront, clampServo, rotationServo);

    }

    @Override
    public void loop() {



    }
}
