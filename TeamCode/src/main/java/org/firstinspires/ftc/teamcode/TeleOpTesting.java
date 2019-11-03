package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "testTeleOP")
public class TeleOpTesting extends OpMode {

    DcMotor leftEncoder;
    DcMotor rightEncoder;
    DcMotor backEncoder;
    DcMotor rightBack;
    DcMotor extusion;
    DcMotor intakeLeft;
    DcMotor intakeRight;

    Servo clampServo;
    Servo rotationServo;
    Servo buildLeft;
    Servo buildRight;

    IntakeOutakeDriving ex;

    @Override
    public void init() {
        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
        backEncoder = hardwareMap.dcMotor.get("backEncoder");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        extusion = hardwareMap.dcMotor.get("extrusion");

        clampServo = hardwareMap.servo.get("clamp");
        rotationServo = hardwareMap.servo.get("rotation");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        /*
        buildLeft = hardwareMap.servo.get("buildLeft");
        buildRight = hardwareMap.servo.get("buildRight");

         */

        ex = new IntakeOutakeDriving(extusion, gamepad1, gamepad2, leftEncoder, backEncoder, rightBack, rightEncoder, clampServo, rotationServo, intakeLeft, intakeRight, buildLeft, buildRight);
        telemetry.addData("initailized: ", true);
        telemetry.addData("Extrusion: ", extusion.getCurrentPosition());
        telemetry.addData("Level:", ex.getLevel());
        telemetry.update();
    }

    @Override
    public void loop() {
        /*
        ex.driving();
        ex.clampBlock(gamepad2.left_bumper, gamepad2.left_trigger > .2);
        ex.clampBuildPlate(gamepad1.left_trigger > 0.2, gamepad1.left_bumper);
        if(gamepad2.a){
            ex.extrusionAuto();
        }
        ex.extrusionManual();
        ex.placeBlockAuto(gamepad2.x);
        ex.placeBlockManual();
        ex.intakeManual();
        if(gamepad2.b) {
            ex.resetExtrusion();
        }
        if(g2.y){
            ex.extrudeToLevel();
        }
        ex.changeLevel(gamepad2.dpad_up, gamepad2.dpad_down);
         */
        ex.placeBlockManual();
        ex.clampBlock(gamepad2.left_bumper, gamepad2.left_trigger > .2);
        ex.driving();
        ex.extrusionManual();
        ex.intakeManual();
        telemetry.addData("EX", ex.getExtrusion());
        telemetry.update();
    }
}
