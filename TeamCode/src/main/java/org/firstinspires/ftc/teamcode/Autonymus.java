package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.lang.*;





@Autonomous(name="Autonymus")
public class Autonymus extends LinearOpMode {


    //VARIABLES
    //
    //
    //
    //

    //location ,location, location
    //robot staring location
    private int robotX = 0;
    private int robotY = 0;
    //robot direction facing
    private int direction = 0;
    private int Case = 0;

    //declaring motors
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;

    //ticks per revolution
    static final double MOTOR_TICK_COUNT = 537.6;

    //wheel diameter cm
    static final int WHEEL_DIAMETER = 10;

    //circumference of wheels in centimeters
    static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;

    //Bot radius as in distance from center point to wheels
    static final int ROBOT_RADIOUS = 16;

    //Power limit
    static final double POWER_FACTOR = .3;

    //number of rotations of wheels to move wheel (circumference) at 45degree strafe
    static final int STRAFE_FACTOR = 2;

    //gear ratio
    static final double GEER_RATIO = 4/3;


    //SHORTCUTS
    //
    //
    //
    //

    public int distanceToTics(double d) {
        //d is centimeters want to travel
        return (int) (d / WHEEL_CIRCUMFRENCE * MOTOR_TICK_COUNT * GEER_RATIO);
    }



    public void rotate(int degrees) {
        //rotates robot specific degrees requested

        double ditsagacne = ROBOT_RADIOUS * 2 * Math.PI * degrees / 360;

        leftFront.setTargetPosition((int) ditsagacne * STRAFE_FACTOR);
        rightFront.setTargetPosition((int) ditsagacne * STRAFE_FACTOR);
        rightBack.setTargetPosition((int) ditsagacne * STRAFE_FACTOR);
        leftBack.setTargetPosition((int) ditsagacne * STRAFE_FACTOR);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(POWER_FACTOR);
        rightFront.setPower(POWER_FACTOR);
        rightBack.setPower(POWER_FACTOR);
        leftBack.setPower(POWER_FACTOR);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {

        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        direction = direction + degrees;
        while (direction < 0 || 360 < direction) {
            if (direction < 0) {
                direction += 360;
            } else {
                direction -= 360;
            }
        }
    }

    public void rotateTo(int degrees) {
        rotate(degrees - direction);
    }


    public void initalize() {
        //declaring motors
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftBack = hardwareMap.dcMotor.get("leftBack");

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
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    public void testMotors(){
        leftFront.setTargetPosition(distanceToTics(10));
        rightFront.setTargetPosition(distanceToTics(10));
        rightBack.setTargetPosition(distanceToTics(10));
        leftBack.setTargetPosition(distanceToTics(10));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.2);
        while(leftFront.isBusy()){
            try{
                Thread.sleep(80);
            }catch(Exception e){
                System.out.print(e);
            }
        }
        rightFront.setPower(.2);
        while(rightFront.isBusy()){
            try{
                Thread.sleep(80);
            }catch(Exception e){
                System.out.print(e);
            }
        }
        rightBack.setPower(.2);
        while(rightBack.isBusy()){
            try{
                Thread.sleep(80);
            }catch(Exception e){
                System.out.print(e);
            }
        }
        leftBack.setPower(.2);
        while(leftBack.isBusy()){
            try{
                Thread.sleep(80);
            }catch(Exception e){
                System.out.print(e);
            }
        }

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void testEncoders() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        for (int i = 0; i <1000; i++){
            double lb = leftBack.getCurrentPosition();
            double rb = rightBack.getCurrentPosition();
            double lf = leftFront.getCurrentPosition();
            double rf = rightFront.getCurrentPosition();
            telemetry.addData("Left Front",lf  );
            telemetry.addData("Left Back", lb);
            telemetry.addData("Right Front", rf);
            telemetry.addData("Right Back", rb);
            telemetry.update();
            try{
                Thread.sleep(80);
            }catch(Exception e){
                System.out.print(e);
            }
        }
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void move(int x, int y) {
        // x and y are distances in centimeters


        //creating variables for distances each motor needs to travel and power levels and special stuff
        int distanceLF;
        int distanceLB;
        int distanceRF;
        int distanceRB;

        double PowerLF;
        double PowerLB;
        double PowerRF;
        double PowerRB;

        double specialPower = 0;
        int Specialdistanjce = 0;


        //if no horizontal movement is necessary
        if (x == 0) {
            distanceLF = STRAFE_FACTOR * y;
            distanceLB = STRAFE_FACTOR * y;
            distanceRF = STRAFE_FACTOR * y;
            distanceRB = STRAFE_FACTOR * y;

            PowerLB = POWER_FACTOR;
            PowerLF = POWER_FACTOR;
            PowerRB = POWER_FACTOR;
            PowerRF = POWER_FACTOR;

        } else {
            //calculates how much each motor needs to move
            Specialdistanjce = (int)(Math.sqrt((x * x) + (y * y)));
            // currently wronmg needs to be fixed
            specialPower = (Math.atan(y/x)/45) - 1;

            if(Case == 1){
                //motor power levels
                PowerRF = POWER_FACTOR;
                PowerLB = POWER_FACTOR;
                PowerLF = specialPower * POWER_FACTOR;
                PowerRB = specialPower * POWER_FACTOR;

                //set motor diances
                distanceRF = Specialdistanjce;
                distanceLB = Specialdistanjce;
                distanceLF = (int) (Specialdistanjce * specialPower);
                distanceRB = (int) (Specialdistanjce * specialPower);
            }else if(Case == 2){
                //motor power levels
                PowerLF = POWER_FACTOR;
                PowerRB = POWER_FACTOR;
                PowerRF = specialPower * POWER_FACTOR;
                PowerLB = specialPower * POWER_FACTOR;

                //set motor diances
                distanceLF = Specialdistanjce;
                distanceRB = Specialdistanjce;
                distanceRF = (int) (Specialdistanjce * specialPower);
                distanceLB = (int) (Specialdistanjce * specialPower);
            }else if(Case == 3){
                //motor power levels
                PowerLF = POWER_FACTOR;
                PowerRB = POWER_FACTOR;
                PowerRF = -specialPower * POWER_FACTOR;
                PowerLB = -specialPower * POWER_FACTOR;

                //set motor diances
                distanceLF = Specialdistanjce;
                distanceRB = Specialdistanjce;
                distanceRF = (int) (Specialdistanjce * specialPower);
                distanceLB = (int) (Specialdistanjce * specialPower);
            }else if(Case == 4){
                //motor power levels
                PowerRF = -POWER_FACTOR;
                PowerLB = -POWER_FACTOR;
                PowerLF = specialPower * POWER_FACTOR;
                PowerRB = specialPower * POWER_FACTOR;

                //set motor diances
                distanceRF = Specialdistanjce;
                distanceLB = Specialdistanjce;
                distanceLF = (int) (Specialdistanjce * specialPower);
                distanceRB = (int) (Specialdistanjce * specialPower);
            }else if(Case == 5){
                //motor power levels
                PowerLF = -POWER_FACTOR;
                PowerRB = -POWER_FACTOR;
                PowerRF = -specialPower * POWER_FACTOR;
                PowerLB = -specialPower * POWER_FACTOR;

                //set motor diances
                distanceLF = Specialdistanjce;
                distanceRB = Specialdistanjce;
                distanceRF = (int) (Specialdistanjce * specialPower);
                distanceLB = (int) (Specialdistanjce * specialPower);
            }else if(Case == 6){
                //motor power levels
                PowerRF = -POWER_FACTOR;
                PowerLB = -POWER_FACTOR;
                PowerLF = -specialPower * POWER_FACTOR;
                PowerRB = -specialPower * POWER_FACTOR;

                //set motor diances
                distanceRF = Specialdistanjce;
                distanceLB = Specialdistanjce;
                distanceLF = (int) (Specialdistanjce * specialPower);
                distanceRB = (int) (Specialdistanjce * specialPower);
            }else if(Case == 7){
                //motor power levels
                PowerLF = -POWER_FACTOR;
                PowerRB = -POWER_FACTOR;
                PowerRF = specialPower * POWER_FACTOR;
                PowerLB = specialPower * POWER_FACTOR;

                //set motor diances
                distanceLF = Specialdistanjce;
                distanceRB = Specialdistanjce;
                distanceRF = (int) (Specialdistanjce * specialPower);
                distanceLB = (int) (Specialdistanjce * specialPower);
            }else{
                //motor power levels
                PowerRF = POWER_FACTOR;
                PowerLB = POWER_FACTOR;
                PowerLF = -specialPower * POWER_FACTOR;
                PowerRB = -specialPower * POWER_FACTOR;

                //set motor diances
                distanceRF = Specialdistanjce;
                distanceLB = Specialdistanjce;
                distanceLF = (int) (Specialdistanjce * specialPower);
                distanceRB = (int) (Specialdistanjce * specialPower);
            }

        }


        //calculates number of tics necessary and tells motors to go that many
        leftFront.setTargetPosition(distanceToTics(distanceLF));
        rightFront.setTargetPosition(distanceToTics(distanceRF));
        rightBack.setTargetPosition(distanceToTics(distanceRB));
        leftBack.setTargetPosition(distanceToTics(distanceLB));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(PowerLF);
        rightFront.setPower(PowerRF);
        rightBack.setPower(PowerRB);
        leftBack.setPower(PowerLB);

        while (rightFront.isBusy() && leftBack.isBusy() && leftFront.isBusy() && rightBack.isBusy()) {
            try{
                Thread.sleep(80);
            }catch(Exception e){
                System.out.print(e);
            }
        }

        //stops everything
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);

        //Resets
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //tells us new location
        robotX += x;
        robotY += y;


    }

    public void moveTo(int x, int y) {
        //moves to desired x,y
        double slope;
        if(x == 0){
            slope = 0;
        }else{
            slope = (double)y / (double)x;
        }

        if (-1 <= slope && slope < 1) {
            if (y >= 0) {
                if(x < 0){
                    Case = 1;
                }else{
                    Case = 2;
                }

            } else {
                if(x < 0){
                    Case = 6;
                }else{
                    Case = 5;
                }

            }
        } else {
            if (x > 0) {
                if(y < 0){
                    Case = 7;
                }else{
                    Case = 8;
                }

            } else {
                if(y < 0){
                    Case = 4;
                }else{
                    Case = 3;
                }

            }
        }
        x = Math.abs(x);
        y = Math.abs(y);
        int placehgolder;
        if(x > y){
            placehgolder = x;
            x = y;
            y = placehgolder;
        }
        move(x, y);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initalize();

        waitForStart();
        telemetry.addData("Status", "Started");
        telemetry.update();
        Thread.sleep(100);
        moveTo(20, 30);

        telemetry.addData("Status", "Done");
        telemetry.update();
    }

}




