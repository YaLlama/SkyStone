package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="testing")
public class Testijng extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SKIRT swerve = new SKIRT();
        DcMotor RF = hardwareMap.dcMotor.get("rightFront");
        DcMotor RB = hardwareMap.dcMotor.get("rightBack");
        DcMotor LF = hardwareMap.dcMotor.get("leftFront");
        DcMotor LB = hardwareMap.dcMotor.get("leftBack");
        swerve.initalize(RF, RB, LF, LB, 0.6);
        telemetry.addData("initalized: ", true);
        waitForStart();
        swerve.move(245, 260);

    }
}
