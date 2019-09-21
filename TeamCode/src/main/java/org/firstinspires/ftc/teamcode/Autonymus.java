package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

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

    // case for movement command
    private int Case = 0;

    // gyro things
    private BNO055IMU hero;

    //declaring motors
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;

    //importing other stuff

    //ticks per motor full rotation
    static final double MOTOR_TICK_COUNT = 537.6;

    //wheel diameter cm
    static final int WHEEL_DIAMETER = 10;

    //circumference of wheels in centimeters
    static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;

    //Power limit
    static final double POWER_FACTOR = .6;

    //number of rotations of wheels to move wheel (circumference) at 45degree strafe
    static final int STRAFE_FACTOR = 2;

    //gear ratio wheel divided by motor
    static final double GEER_RATIO = 4/3;


    //SHORTCUTS
    //
    //
    //
    //

    public int distanceToTics(double d) {
        //d is centimeters want to travel
        //divides the distance yoi want to got by circumfrance to get number of motor rotations necessary then converts to number to tics needed to do so
        return (int) (d / WHEEL_CIRCUMFRENCE * MOTOR_TICK_COUNT * GEER_RATIO);
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

        //IMU stuff
        BNO055IMU.Parameters gyro_parameters = new BNO055IMU.Parameters();
        gyro_parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro_parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro_parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample Opmode
        gyro_parameters.loggingEnabled = true;
        gyro_parameters.loggingTag = "IMU";
        gyro_parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        hero = hardwareMap.get(BNO055IMU.class, "hero"); //The name of our heroic IMU
        hero.initialize(gyro_parameters);
        hero.startAccelerationIntegration(new Position(), new Velocity(), 1000);

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

        double specialPower;
        int Specialdistanjce;


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
            specialPower = Math.toDegrees(Math.atan((double)y/(double)x))/45 - 1;

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
                distanceRF = (int) -(Specialdistanjce * specialPower);
                distanceLB = (int) -(Specialdistanjce * specialPower);
            }else if(Case == 4){
                //motor power levels
                PowerRF = -POWER_FACTOR;
                PowerLB = -POWER_FACTOR;
                PowerLF = specialPower * POWER_FACTOR;
                PowerRB = specialPower * POWER_FACTOR;

                //set motor diances
                distanceRF = -Specialdistanjce;
                distanceLB = -Specialdistanjce;
                distanceLF = (int) (Specialdistanjce * specialPower);
                distanceRB = (int) (Specialdistanjce * specialPower);
            }else if(Case == 5){
                //motor power levels
                PowerLF = -POWER_FACTOR;
                PowerRB = -POWER_FACTOR;
                PowerRF = -specialPower * POWER_FACTOR;
                PowerLB = -specialPower * POWER_FACTOR;

                //set motor diances
                distanceLF = -Specialdistanjce;
                distanceRB = -Specialdistanjce;
                distanceRF = (int) -(Specialdistanjce * specialPower);
                distanceLB = (int) -(Specialdistanjce * specialPower);
                telemetry.addData("special power", PowerRF);
                telemetry.addData("short distance", distanceLF);
                telemetry.addData("long distance", distanceLB);
                telemetry.update();

            }else if(Case == 6){
                //motor power levels
                PowerRF = -POWER_FACTOR;
                PowerLB = -POWER_FACTOR;
                PowerLF = -specialPower * POWER_FACTOR;
                PowerRB = -specialPower * POWER_FACTOR;

                //set motor diances
                distanceRF = -Specialdistanjce;
                distanceLB = -Specialdistanjce;
                distanceLF = (int) -(Specialdistanjce * specialPower);
                distanceRB = (int) -(Specialdistanjce * specialPower);
            }else if(Case == 7){
                //motor power levels
                PowerLF = -POWER_FACTOR;
                PowerRB = -POWER_FACTOR;
                PowerRF = specialPower * POWER_FACTOR;
                PowerLB = specialPower * POWER_FACTOR;

                //set motor diances
                distanceLF = -Specialdistanjce;
                distanceRB = -Specialdistanjce;
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
                distanceLF = (int) -(Specialdistanjce * specialPower);
                distanceRB = (int) -(Specialdistanjce * specialPower);
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
                    telemetry.addData("case", 1);
                }else{
                    Case = 2;
                    telemetry.addData("case", 2);
                }

            } else {
                if(x < 0){
                    Case = 6;
                    telemetry.addData("case", 6);
                }else{
                    Case = 5;
                    telemetry.addData("case", 5);
                }

            }
        } else {
            if (x > 0) {
                if(y < 0){
                    Case = 7;
                    telemetry.addData("case", 7);
                }else{
                    Case = 8;
                    telemetry.addData("case", 8);
                }

            } else {
                if(y < 0){
                    Case = 4;
                    telemetry.addData("case", 4);
                }else{
                    Case = 3;
                    telemetry.addData("case", 3);
                }

            }
        }
        telemetry.update();
        x = Math.abs(x);
        y = Math.abs(y);
        int placehgolder;
        if(false){
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
        moveTo(-120, 0);
        moveTo(120, 0);

        telemetry.addData("Status", "Done");
        telemetry.update();
    }

}




