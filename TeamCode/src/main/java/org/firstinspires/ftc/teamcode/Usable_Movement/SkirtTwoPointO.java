package org.firstinspires.ftc.teamcode.Usable_Movement;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SkirtTwoPointO {

    //ticks per motor full rotation
    static final double MOTOR_TICK_COUNT = 537.6;

    //wheel diameter cm
    static final int WHEEL_DIAMETER = 10;

    //circumference of wheels in centimeters
    static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;

    //gear ratio wheel divided by motor
    static final double GEER_RATIO = 4/3;

    double disP;

    double slope;

    double specialPower;

    double correction;

    int encoderdistance;

    int distanceToPoint;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftBack;

    double PowerLB;
    double PowerLF;
    double PowerRB;
    double PowerRF;

    public SkirtTwoPointO(DcMotor lb, DcMotor lf, DcMotor rb, DcMotor rf){
        rightBack = rb;
        rightFront = rf;
        leftBack = lb;
        leftFront = lf;

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

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int distanceToTics(double d) {
        //d is centimeters want to travel
        //divides the distance yoi want to got by circumfrance to get number of motor rotations necessary then converts to number to tics needed to do so
        return (int) (d / WHEEL_CIRCUMFRENCE * MOTOR_TICK_COUNT * GEER_RATIO);
    }

    public void move(int x, int y, double power, int threshholdPerAxiz, int rotationThreshhold) {
        // x and y are distances in encoder tics

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        distanceToPoint = (int) (Math.sqrt((x * x) + (y * y)));

        distanceToPoint = (int) (distanceToPoint + distanceToPoint * 3 * (1 - specialPower));

        //if no horizontal movement is necessary
        if(y > 0){
            disP = power;
        }else{
            disP = -power;
        }
        if (x == 0) {
            PowerLB = disP;
            PowerLF = disP;
            PowerRB = disP;
            PowerRF = disP;
        } else {
            slope = (double) y / (double) x;

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

        encoderdistance = distanceToTics(distanceToPoint + (int) (distanceToPoint * specialPower));

        correction = disP / 7;

        while(rightFront.getCurrentPosition() + rightBack.getCurrentPosition() < encoderdistance || leftBack.getCurrentPosition() + leftFront.getCurrentPosition() < encoderdistance){

            //calculates number of tics necessary and tells motors to go that many
            leftFront.setPower(PowerLF);
            rightFront.setPower(PowerRF);
            rightBack.setPower(PowerRB);
            leftBack.setPower(PowerLB);

            if (Math.abs(rightFront.getCurrentPosition() + rightBack.getCurrentPosition() - (leftBack.getCurrentPosition() + leftFront.getCurrentPosition())) > rotationThreshhold) {
                leftFront.setPower(PowerLF);
                rightFront.setPower(PowerRF);
                rightBack.setPower(PowerRB);
                leftBack.setPower(PowerLB);
            } else if (rightFront.getCurrentPosition() + rightBack.getCurrentPosition() < leftBack.getCurrentPosition() + leftFront.getCurrentPosition()) {
                rightFront.setPower(PowerRF + correction);
                rightBack.setPower(PowerRB + correction);

                PowerRF = PowerRF + correction;
                PowerRB = PowerRB + correction;

            } else {
                leftBack.setPower(PowerLB + correction);
                leftFront.setPower(PowerLF + correction);

                PowerLB = PowerLB + correction;
                PowerLF = PowerLF + correction;
            }
        }
        while(rightFront.getCurrentPosition() + rightBack.getCurrentPosition() < leftBack.getCurrentPosition() + leftFront.getCurrentPosition()){
            PowerRF = correction;
            PowerRB = correction;
            PowerLB = -correction;
            PowerLF = -correction;
        }
        while(rightFront.getCurrentPosition() + rightBack.getCurrentPosition() > leftBack.getCurrentPosition() + leftFront.getCurrentPosition()){
            PowerRF = -correction;
            PowerRB = -correction;
            PowerLB = correction;
            PowerLF = correction;
        }

        //stops everything
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);

    }
}
