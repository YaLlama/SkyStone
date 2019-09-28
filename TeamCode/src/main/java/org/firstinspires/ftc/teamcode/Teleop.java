package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Teleop", group="MecanumDrive")
public class Teleop extends OpMode {
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;

    double m1, m2, m3, m4;
    double x1, x2, y1, y2, s1, s2, s3;

    private double rtrigpos = 0;
    private double lefty = 0;
    private double righty = 0;
    private double lastrighty = 0;
    private boolean dpadtouch = false;
    private boolean apress = false;
    //s1 anmd s2w are slowing variables.
    @Override
    public void init() {
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");


        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFrontMotor.setPower(0); // Set power to the 4 drive motors (on init)
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);


    }

    @Override
    public void init_loop() {
        /*telemetry.addData("y1", y1);
        telemetry.addData("y2", y2);
        telemetry.addData("x1", x1);
        telemetry.addData("x2", x2);
        telemetry.addData("fl", m1);
        telemetry.addData("bl", m2);
        telemetry.addData("rf ", m3);
        telemetry.addData("rb", m4);*/


    }

    @Override
    public void start() {
        //suggested that servo positions are initialized here

    }

    @Override
    public void loop() {
        s1 = 1;
        s2 = 1;

        if (gamepad1.right_trigger > 0.2) {
            s3 = .6;
        }
        if (gamepad1.right_bumper) {
            s3 = .4;
        } else {
            s3 = 0.8;
        }

        y2 = -gamepad1.left_stick_y;
        y1 = -gamepad1.right_stick_y;
        x1 = gamepad1.right_stick_x;
        x2 = gamepad1.left_stick_x;

        m1 = (-y1 - x1) * s1 * s2 * s3;
        m2 = (-y1 + x1) * s1 * s2 * s3;
        m3 = (y2 - x2) * s1 * s2 * s3;
        m4 = (y2 + x2) * s1 * s2 * s3;
        leftFrontMotor.setPower((-y1 - x1) * s2 * s3);
        leftBackMotor.setPower((-y1 + x1) * s2 * s3);
        rightFrontMotor.setPower((y2 - x2) * s1 * s3);
        rightBackMotor.setPower((y2 + x2) * s1 * s3);

    }

    public void stop() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}