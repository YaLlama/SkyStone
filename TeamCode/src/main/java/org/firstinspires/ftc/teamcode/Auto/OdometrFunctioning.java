package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class OdometrFunctioning {

    DcMotor l;
    DcMotor r;
    DcMotor b;

    int lMulti;
    int rMulti;
    int bMulti;

    public OdometrFunctioning(DcMotor leftOmni, DcMotor rightOmni, DcMotor backOmni, int lfReversed, int rfReversed, int bReversed){
        l = leftOmni;
        r = rightOmni;
        b = backOmni;

        lMulti = lfReversed;
        rMulti = rfReversed;
        bMulti = bReversed;
    }

    public int[] location(){
        int[] location = {(l.getCurrentPosition() * lMulti + r.getCurrentPosition() * rMulti) / 2, b.getCurrentPosition() * bMulti};
        return location;
    }

    public double correction(double power){
        power = (power / 3) * (l.getCurrentPosition() - r.getCurrentPosition()) / 100;
        return power;
    }

}
