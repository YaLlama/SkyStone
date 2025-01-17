package org.firstinspires.ftc.teamcode.Usable_Movement;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="testing")
public class Testijng extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SKIRT ct = new SKIRT();
        telemetry.addData("working", true);
        telemetry.update();
        DcMotor RF = hardwareMap.dcMotor.get("rightFront");
        DcMotor RB = hardwareMap.dcMotor.get("rightBack");
        DcMotor LF = hardwareMap.dcMotor.get("leftFront");
        DcMotor LB = hardwareMap.dcMotor.get("leftBack");
        ct.initalize(RF, RB, LF, LB);
        telemetry.addData("initalized: ", true);
        waitForStart();
        ct.move(45, 45, .3);
    }
}
