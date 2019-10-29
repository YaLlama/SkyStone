package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Extrusion {

    //highest encoder value extrusion can be
    static final int MAX_HEIGHT = 0;
    //Max extrusion motor power
    static final double EXTRUSION_POWER = 0;

    //amount in encoder tics below the snappin point the threshhold is
    static final int BOTTOM_SNAP = 0;

    private static DcMotor ex;
    private static Gamepad g2;

    private boolean autonymus = false;

    //encoder value with first block placed perfectly on build plate
    public static final int FIRST_LEVEL_HEIGHT = 0;

    //amount of encoder value needed to be added to clear block trying to stack on
    public static final int CLEARENCE = 0;

    public void init(DcMotor extrusion, Gamepad gamePad2){
        ex = extrusion;
        g2 = gamePad2;
        ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void extrusionManual(){
        //Control for moving extrusion up
        autonymus = false;
        if(g2.dpad_up && ex.getCurrentPosition() < MAX_HEIGHT){
            ex.setPower(EXTRUSION_POWER);
        }else if(g2.dpad_down && ex.getCurrentPosition() > 0){
            ex.setPower(-EXTRUSION_POWER);
        }else {
            ex.setPower(0);
        }
    }

    public void extrusionAuto(){
        //control for locking to closest height
        if(g2.x){
            autonymus = true;
            ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //snapping
            ex.setTargetPosition((ex.getCurrentPosition() - FIRST_LEVEL_HEIGHT - BOTTOM_SNAP) / CLEARENCE);
            ex.setPower(EXTRUSION_POWER);
            //trying out get power float dont know how it works
        }else if(!autonymus || ex.getPowerFloat()) {
            ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ex.setPower(0);
        }
    }
}
