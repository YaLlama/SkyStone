package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="testing")
public class Testijng extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Conttuiuys ct = new Conttuiuys();
        ct.Main();
        telemetry.addData("working", true);
        telemetry.update();
        DcMotor RF = hardwareMap.dcMotor.get("rightFront");
        DcMotor RB = hardwareMap.dcMotor.get("rightBack");
        DcMotor LF = hardwareMap.dcMotor.get("leftFront");
        DcMotor LB = hardwareMap.dcMotor.get("leftBack");
        ct.initalize(RF, RB, LF, LB);
        telemetry.addData("initalized: ", true);
        waitForStart();
        Conttuiuys.x = 1;
        Conttuiuys.y = 1;
        Conttuiuys.POWER_FACTOR = .3;
        Conttuiuys.running = true;
        try{
            Thread.sleep(2000);
        }catch(Exception e){

        }
        Conttuiuys.running = false;
        Conttuiuys.TOTAL_RUNNING = false;
    }
}
