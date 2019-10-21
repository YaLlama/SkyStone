package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SKRTOdometry {

    //VARIABLES
    //
    //
    //
    //

    //declaring motors
    public static DcMotor rightFront;
    public static DcMotor leftFront;
    public static DcMotor rightBack;
    public static DcMotor leftBack;

    Odometer odo;


    double PowerLF = 0;
    double PowerLB = 0;
    double PowerRF = 0;
    double PowerRB = 0;

    double specialPower;

    double strafeFactor;

    //robot readious, distance from center of a wheel to center of the robot
    static final double ROBOT_RADIOUS = 20;




    //SHORTCUTS / COMMANDS
    //
    //
    //
    //
    public SKRTOdometry(DcMotor RFE, DcMotor LFE, DcMotor LBE, DcMotor RB) {
        //declaring motors
        rightFront = RFE;
        leftFront = LFE;
        rightBack = RB;
        leftBack = LBE;

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setting motors to fun with encoders
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);

        odo = new Odometer(rightFront, leftFront, leftBack, -1, -1, -1);
        odo.initializeOdometry();

    }

    public int posX(){
        return (int) odo.getposition()[0];
    }
    public int posY(){
        return (int) odo.getposition()[1];
    }

    public void move(int x, int y, double power, int threshholdPerAxiz) {
        // x and y are distances in centimeters
        int DisX;
        int DisY;

        int SrtX = (int) odo.getposition()[0];
        int SrtY = (int) odo.getposition()[1];

        int CurX;
        int CurY;

        double disP;

        //moves to desired x,y
        double slope;

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        do {
            CurX = (int) odo.getposition()[0] - SrtX;
            CurY = (int) odo.getposition()[1] - SrtY;

            DisX = x - CurX;
            DisY = y - CurY;

            odo.updateOdometry();

            //if no horizontal movement is necessary
            if(DisY < 0){
                disP = -power;
            }else{
                disP = power;
            }
            if (DisX == 0) {
                PowerLB = disP;
                PowerLF = disP;
                PowerRB = disP;
                PowerRF = disP;
            } else {
                slope = (double) DisY / (double) DisX;

                specialPower = Math.abs(Math.toDegrees(Math.atan(slope))) / 45 - 1;

                if(slope > 0){
                    PowerRF = disP * specialPower;
                    PowerLB = disP * specialPower;
                    PowerRB = disP;
                    PowerLF = disP;
                }else{
                    PowerLF = disP * specialPower;
                    PowerRB = disP * specialPower;
                    PowerLB = disP;
                    PowerRF = disP;
                }

            }


            //calculates number of tics necessary and tells motors to go that many
            leftFront.setPower(PowerLF);
            rightFront.setPower(PowerRF);
            rightBack.setPower(PowerRB);
            leftBack.setPower(PowerLB);

        }while(Math.abs(DisX) > threshholdPerAxiz && Math.abs(DisY) > threshholdPerAxiz);


        //stops everything
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);

    }

    public void moveTo(int x, int y, double power, int threshholdPerAxiz) {
        // x and y are distances in centimeters
        int DisX;
        int DisY;

        double disP;

        //moves to desired x,y
        double slope;

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        do {
            DisX = x - (int)odo.getposition()[0];
            DisY = y - (int)odo.getposition()[1];

            odo.updateOdometry();

            //if no horizontal movement is necessary
            if(DisY < 0){
                disP = -power;
            }else{
                disP = power;
            }
            if (DisX == 0) {
                PowerLB = disP;
                PowerLF = disP;
                PowerRB = disP;
                PowerRF = disP;
            } else {
                slope = (double) DisY / (double) DisX;

                specialPower = Math.abs(Math.toDegrees(Math.atan(slope))) / 45 - 1;

                if(slope > 0){
                    PowerRF = disP * specialPower;
                    PowerLB = disP * specialPower;
                    PowerRB = disP;
                    PowerLF = disP;
                }else{
                    PowerLF = disP * specialPower;
                    PowerRB = disP * specialPower;
                    PowerLB = disP;
                    PowerRF = disP;
                }

            }


            //calculates number of tics necessary and tells motors to go that many
            leftFront.setPower(PowerLF);
            rightFront.setPower(PowerRF);
            rightBack.setPower(PowerRB);
            leftBack.setPower(PowerLB);

        }while(Math.abs(DisX) > threshholdPerAxiz && Math.abs(DisY) > threshholdPerAxiz);


        //stops everything
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);

    }
    public void moveTo(int x, int y, double power, int threshholdPerAxiz, int degrees, int andgleThreshhold) {
        // x and y are distances in centimeters
        int DisX;
        int DisY;

        int DegD;

        double correction = power/6;

        double correct;

        double disP;

        //moves to desired x,y
        double slope;

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        correct = 0;

        do {
            DisX = x - (int)odo.getposition()[0];
            DisY = y - (int)odo.getposition()[1];

            DegD = degrees - (int) odo.getHeadingDeg();

            odo.updateOdometry();

            if(DegD == 0){
                correct = 0;
            }else if(DegD > 0){
                correct = correction;
            }else{
                correct = -correction;
            }

            //if no horizontal movement is necessary
            if(DisY < 0){
                disP = -power;
            }else{
                disP = power;
            }
            if (DisX == 0) {
                if(DisY == 0){
                    PowerLB = correct;
                    PowerLF = correct;
                    PowerRB = -correct;
                    PowerRF = -correct;
                }else {
                    PowerLB = disP + correct;
                    PowerLF = disP + correct;
                    PowerRB = disP - correct;
                    PowerRF = disP - correct;
                }
            } else {
                slope = Math.tan(Math.atan((double) DisY / (double) DisX) - Math.toRadians((double)DegD));

                specialPower = Math.abs(Math.toDegrees(Math.atan(slope))) / 45 - 1;

                if(slope > 0){
                    PowerRF = disP * specialPower - correct;
                    PowerLB = disP * specialPower + correct;
                    PowerRB = disP - correct;
                    PowerLF = disP + correct;
                }else{
                    PowerLF = disP * specialPower + correct;
                    PowerRB = disP * specialPower - correct;
                    PowerLB = disP + correct;
                    PowerRF = disP - correct;
                }

            }


            //calculates number of tics necessary and tells motors to go that many
            leftFront.setPower(PowerLF);
            rightFront.setPower(PowerRF);
            rightBack.setPower(PowerRB);
            leftBack.setPower(PowerLB);

        }while(Math.abs(DisX) > threshholdPerAxiz && Math.abs(DisY) > threshholdPerAxiz && Math.abs(DegD) > andgleThreshhold);


        //stops everything
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);



    }
    public void moveTo(int x, int y, double power, int  threshholdPerAxiz, double degrees, int DO_NOT_USE_THIS_ONE_STILL_IN_BETA){

        // x and y are distances in centimeters
        int DisX;
        int DisY;
        double DegD;
        double DisP;

        //moves to desired x,y
        double slope;

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        do {
            DisX = x - (int)odo.getposition()[0];
            DisY = y - (int)odo.getposition()[1];

            DegD = degrees - odo.getHeadingDeg();


            odo.updateOdometry();

            //if no horizontal movement is necessary
            slope = Math.tan(Math.atan((double) DisY / (double) DisX) + DegD);

            specialPower = Math.abs(Math.toDegrees(Math.atan(slope))) / 45 - 1;

            strafeFactor = 28424.460675 * ROBOT_RADIOUS * DegD / (28424.460675 * ROBOT_RADIOUS * DegD + 1628601.631621 * Math.sqrt(x^2 + y^2));

            if(DisY < 0){
                DisP = -power;
            }else{
                DisP = power;
            }

            if (DisX == 0) {
                PowerLB = DisP + strafeFactor;
                PowerLF = DisP + strafeFactor;
                PowerRB = DisP - strafeFactor;
                PowerRF = DisP - strafeFactor;

            } else {

                if(slope > 0){
                    PowerRF = DisP * specialPower - strafeFactor;
                    PowerLB = DisP * specialPower + strafeFactor;
                    PowerRB = DisP - strafeFactor;
                    PowerLF = DisP + strafeFactor;
                }else{
                    PowerLF = DisP * specialPower + strafeFactor;
                    PowerRB = DisP * specialPower - strafeFactor;
                    PowerLB = DisP + strafeFactor;
                    PowerRF = DisP - strafeFactor;
                }

            }

            //calculates number of tics necessary and tells motors to go that many

            leftFront.setPower(PowerLF);
            rightFront.setPower(PowerRF);
            rightBack.setPower(PowerRB);
            leftBack.setPower(PowerLB);

        }while(Math.abs(DisX) > threshholdPerAxiz || Math.abs(DisY) > threshholdPerAxiz);

        //stops everything
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);


    }

}
