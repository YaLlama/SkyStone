package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleOp {

    //highest encoder value extrusion can be
    static final int MAX_HEIGHT = 0;
    //Max extrusion motor power
    static final double EXTRUSION_POWER = 0;

    //amount in encoder tics below the snappin point the threshhold is
    static final int BOTTOM_SNAP = 0;

    //joystick dead zone
    static final double DEAD_ZONE = 0;

    //servo max and min
    static final double MAX_ROTATION = 0;
    static final double MINIMUM_ROTATION = 0;

    //open position of clamping servo
    static final double OPEN_POSSITION = 0;

    //closed prosition of clamping servo
    static final double CLOSED_POSSITION = 0;

    //clamping of build plate servo values
    static final int BUILD_CLAPED_LEFT = 0;
    static final int BUILD_CLAPED_RIGHT = 0;

    //releasing/ default of build plate servos
    static final int BUILD_UNCLAPED_LEFT = 0;
    static final int BUILD_UNCLAPED_RIGHT = 0;


    private static DcMotor leftFront;
    private static DcMotor rightFront;
    private static DcMotor leftBack;
    private static DcMotor rightBack;
    private static DcMotor ex;
    private static Gamepad g2;
    private static Gamepad g1;
    private static Servo cs;
    private static Servo rs;
    private static DcMotor ir;
    private static DcMotor il;
    private static Servo bl;
    private static Servo br;

    private boolean extrusionAutonymus = false;
    private boolean placingAutonymus = false;

    private boolean step1done = false;
    private boolean step2done = false;


    //encoder value with first block placed perfectly on build plate
    public static final int FIRST_LEVEL_HEIGHT = 0;

    //amount of encoder value needed to be added to clear block trying to stack on
    public static final int CLEARENCE = 0;

    //the encoder amount it needs to be lowered in order to lock block into place
    public static final int LOCKED_POSITION = 0;



    //driving
    final int s1 = 1;
    final int s2 = 1;
    double s3;

    double y1;
    double y2;
    double x1;
    double x2;

    double rx;
    double ry;


    public TeleOp(DcMotor extrusion, Gamepad gamepad1, Gamepad gamePad2, DcMotor lf, DcMotor lb, DcMotor rb, DcMotor rf, Servo clampServo, Servo rotateServo, DcMotor intakeLeft, DcMotor intakeRight, Servo buildLeft, Servo buildRight){
        //Motors not reversed in any way except for intake

        ex = extrusion;
        g2 = gamePad2;
        g1 = gamepad1;

        cs = clampServo;
        rs = rotateServo;

        ir = intakeRight;
        il = intakeLeft;

        bl = buildLeft;
        br = buildRight;

        ir.setDirection(DcMotorSimple.Direction.REVERSE);
        il.setDirection(DcMotorSimple.Direction.FORWARD);

        ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBack = rb;
        rightFront = rf;
        leftBack = lb;
        leftFront = lf;

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        il.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ir.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        il.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


    }

    public void driving(){
        //last year controls
        if (g1.right_trigger > 0.2) {
            s3 = .6;
        }
        if (g1.right_bumper) {
            s3 = .4;
        } else {
            s3 = 0.8;
        }

        y2 = -g1.left_stick_y;
        y1 = -g1.right_stick_y;
        x1 = g1.right_stick_x;
        x2 = g1.left_stick_x;

        leftFront.setPower((-y1 - x1) * s2 * s3);
        leftBack.setPower((-y1 + x1) * s2 * s3);
        rightFront.setPower((y2 - x2) * s1 * s3);
        rightBack.setPower((y2 + x2) * s1 * s3);
    }

    public void extrusionManual(){
        //Control for moving extrusion up

        if(g2.left_stick_y > DEAD_ZONE && ((-g2.left_stick_y > 0 && ex.getCurrentPosition() < MAX_HEIGHT) || (-g2.left_stick_y < 0 && ex.getCurrentPosition() > FIRST_LEVEL_HEIGHT))){
            ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extrusionAutonymus = false;
            ex.setPower(-g2.left_stick_y);
        }else if(!extrusionAutonymus){
            ex.setPower(0);
        }
    }

    public void extrusionAuto(){
        //control for locking to closest height
        if(g2.x) {
            extrusionAutonymus = true;
            ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //snapping
            ex.setTargetPosition((ex.getCurrentPosition() - FIRST_LEVEL_HEIGHT - BOTTOM_SNAP) / CLEARENCE + FIRST_LEVEL_HEIGHT);
            ex.setPower(EXTRUSION_POWER);
        }
    }

    public void placeBlockManual(){
        //Control for moving clamping servos
        if(g2.left_stick_x > DEAD_ZONE && ((g2.left_stick_x > 0 && ex.getCurrentPosition() < MAX_ROTATION) || (g2.left_stick_x < 0 && ex.getCurrentPosition() > MINIMUM_ROTATION))) {
            placingAutonymus = false;
            rs.setPosition(rs.getPosition() + g2.left_stick_y * .1);
        }
    }

    public void placeBlockAuto(){
        //controls for automatically placing block, need to hold x
        step1done = false;
        step2done = false;
        while(g1.x){
            if(!step1done) {
                rs.setPosition(MAX_ROTATION);
                if (rs.getPosition() == MAX_ROTATION) {
                    ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ex.setTargetPosition(ex.getCurrentPosition() - LOCKED_POSITION);
                    step1done = true;
                }
            }
            else if(!step2done) {
                if (ex.getCurrentPosition() == ex.getTargetPosition()) {
                    cs.setPosition(OPEN_POSSITION);
                    if(cs.getPosition() == OPEN_POSSITION){
                        ex.setTargetPosition(ex.getCurrentPosition() + LOCKED_POSITION);
                        step2done = true;
                    }
                }
            }else{
                if(ex.getCurrentPosition() == ex.getTargetPosition()){
                    break;
                }
            }

        }
    }

    public void intakeManual(){
        //intake controls
        if(Math.abs(g2.right_stick_x) > DEAD_ZONE || Math.abs(g2.right_stick_y) > DEAD_ZONE){
            rx = g2.right_stick_x;
            ry = -g2.right_stick_y;
            il.setPower(ry + rx);
            ir.setPower(ry - rx);
        }else{
            il.setPower(0);
            ir.setPower(0);
        }
    }

    public void clampBuildPlate(){
        //clamping build plate controls
        if(g1.right_bumper){
            br.setPosition(BUILD_CLAPED_RIGHT);
            bl.setPosition(BUILD_CLAPED_LEFT);
        }else if(g1.left_bumper){
            br.setPosition(BUILD_UNCLAPED_RIGHT);
            bl.setPosition(BUILD_UNCLAPED_LEFT);
        }
    }

    public void clampBlock(){
        //clmaping block controls
        if(g2.left_bumper){
            cs.setPosition(OPEN_POSSITION);
        }else if(g2.right_bumper){
            cs.setPosition(CLOSED_POSSITION);
        }
    }
    public void resetExtrusion(){
        //comntrol for resetting evertything
        if(g1.a) {
            ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ex.setTargetPosition(0);
            ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rs.setPosition(MINIMUM_ROTATION);
        }
    }

}
