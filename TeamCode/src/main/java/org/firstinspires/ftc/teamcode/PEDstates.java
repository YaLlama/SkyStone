package org.firstinspires.ftc.teamcode;

import java.lang.Math;

public class PEDstates {
    private double kp;
    private double ks;
    private double kd;
    private double ceiling;
    private double floor;

    private double P;
    private double E;
    private double D;

    private double error = 0;
    private double errorsum = 0;
    private double lasterror = 0;
    private double target;
    private double initial;
    private double correction;
    private int time;

    public PEDstates(double p, double s, double d, double ceil, double flr){
        this.kp = p;
        this.ks = s;
        this.kd = d;
        this.ceiling = ceil;
        this.floor = flr;
    }

    public void settarget(double target) {
        this.target = target;
    }

    public void setinitial(double initial){
        this.initial = initial;
    }

    public boolean finished(){
        if((error < 2 && error > -2) || time > 10000000){
            return true;
        }else{
            return false;
        }
    }

    public double getoutput(double reading){
        target += initial;
        error = reading - target;

        P = -error * kp;

        if(time % 10 == 0){
            errorsum += -error;
        }

        E = 0;

        D = (lasterror - error) * kd;

        correction = P + E + D;

        if(correction > ceiling){
            correction = ceiling;
        }else if(correction < floor){
            correction = floor;
        }

        lasterror = error;
        time++;

        return correction;
    }

    public double getTarget(){
        return target;
    }

    public double getError(){
        return error;
    }
}
