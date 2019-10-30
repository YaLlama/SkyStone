package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleOpTesting extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor extusion;
    DcMotor intakeLeft;
    DcMotor intakeRight;

    Servo clampServo;
    Servo rotationServo;
    Servo buildLeft;
    Servo buildRight;

    TeleOp ex;

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        extusion = hardwareMap.dcMotor.get("extrusion");

        clampServo = hardwareMap.servo.get("clamp");
        rotationServo = hardwareMap.servo.get("rotation");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        buildLeft = hardwareMap.servo.get("buildLeft");
        buildRight = hardwareMap.servo.get("buildRight");

        ex = new TeleOp(extusion, gamepad1, gamepad2, leftFront, leftBack, rightBack, rightFront, clampServo, rotationServo, intakeLeft, intakeRight, buildLeft, buildRight);
    }

    @Override
    public void loop() {
        ex.driving();
        ex.clampBlock();
        ex.clampBuildPlate();
        ex.extrusionAuto();
        ex.extrusionManual();
        ex.placeBlockAuto();
        ex.placeBlockManual();
        ex.intakeManual();
        ex.resetExtrusion();
    }
}
