package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeOutakeDriving {

    //highest encoder value extrusion can be
    static final int MAX_HEIGHT = 2300;
    //Max extrusion motor power
    static final double EXTRUSION_POWER = 1;

    //height of extrusion to pick up block
    static final int BLOCK_HEIGHT = 475;

    //height of primed extrusion
    static final int EXTRUSION_PRIME = 600;

    //amount in encoder tics below the snappin point the threshhold is
    static final int BOTTOM_SNAP = 200;

    //joystick dead zone
    static final double DEAD_ZONE = 0;

    //servo max and min
    static final double MAX_ROTATION = 0;
    static final double MINIMUM_ROTATION = .72;

    //open position of clamping servo
    static final double OPEN_POSSITION = .6;

    //closed prosition of clamping servo
    static final double CLOSED_POSSITION = .35;

    //clamping of build plate servo values
    static final double BUILD_CLAPED_LEFT = .35;
    static final double BUILD_CLAPED_RIGHT = .4;

    //releasing/ default of build plate servos
    static final double BUILD_UNCLAPED_LEFT = 0;
    static final double BUILD_UNCLAPED_RIGHT = 0;

    //the speed at which the joystick moves the rotating servo
    static final double JOYSITCK_SPEED = .1;

    //the spead at which the dpad moves the servo
    static final double DEPAD_SPEED = .01;


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
    public static final int FIRST_LEVEL_HEIGHT = 350;

    //the amount of encoder tics between levels
    public static final int LEVEL_HEIGHT = 300;

    //the encoder amount it needs to be lowered in order to lock block into place
    public static final int LOCKED_POSITION = 150;

    //servo primed possition
    public static final double SERVO_PRIME = MAX_ROTATION;



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

    static int level = 0;

    static boolean releasedchangeLevel = true;

    static int targetPossition = 0;

    static boolean releasedblockHeight = true;

    static boolean releasedprime = true;

    static boolean releasedextrudeToLevel = true;


    public IntakeOutakeDriving(DcMotor extrusion, Gamepad gamepad1, Gamepad gamePad2, DcMotor lf, DcMotor lb, DcMotor rb, DcMotor rf, Servo clampServo, Servo rotateServo, DcMotor intakeLeft, DcMotor intakeRight, Servo buildLeft, Servo buildRight){
        //Motors not reversed in any way except for intake

        ex = extrusion;
        g2 = gamePad2;
        g1 = gamepad1;

        cs = clampServo;
        rs = rotateServo;

        cs.setDirection(Servo.Direction.FORWARD);
        rs.setDirection(Servo.Direction.FORWARD);
        cs.setPosition(OPEN_POSSITION);
        rs.setPosition(MINIMUM_ROTATION);

        ir = intakeRight;
        il = intakeLeft;

        bl = buildLeft;
        br = buildRight;

        /*
        br.setDirection(Servo.Direction.FORWARD);
        bl.setDirection(Servo.Direction.REVERSE);

        bl.setPosition(BUILD_UNCLAPED_LEFT);
        br.setPosition(BUILD_UNCLAPED_RIGHT);

         */

        ir.setDirection(DcMotorSimple.Direction.FORWARD);
        il.setDirection(DcMotorSimple.Direction.REVERSE);



        ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ex.setDirection(DcMotorSimple.Direction.FORWARD);

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

        ex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        if(Math.abs(g2.left_stick_y) > DEAD_ZONE && ((-g2.left_stick_y > 0 && ex.getCurrentPosition() < MAX_HEIGHT) || (-g2.left_stick_y < 0 && ex.getCurrentPosition() > 0))) {
            ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extrusionAutonymus = false;
            ex.setPower(-g2.left_stick_y);
        }else if(!extrusionAutonymus){
            ex.setPower(0);
        }
    }

    public void extrusionAuto(){
        extrusionAutonymus = true;
        //snapping
        ex.setTargetPosition((ex.getCurrentPosition() - FIRST_LEVEL_HEIGHT - BOTTOM_SNAP) / LEVEL_HEIGHT + FIRST_LEVEL_HEIGHT);
        ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ex.setPower(EXTRUSION_POWER);
    }

    public void placeBlockManual(){
        //Control for moving clamping servos
        if(g2.dpad_right){
            placingAutonymus = false;
            rs.setPosition(MINIMUM_ROTATION);
        }else if(g2.dpad_left){
            placingAutonymus = false;
            rs.setPosition(MAX_ROTATION);
        }
    }

    public void placeBlockAuto(boolean placeBlock){
        //controls for automatically placing block, need to hold x
        if(placeBlock){
            if(!step1done) {
                rs.setPosition(MAX_ROTATION);
                if (rs.getPosition() == MAX_ROTATION) {
                    targetPossition = targetPossition - LOCKED_POSITION;
                    ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    ex.setPower(-EXTRUSION_POWER / 3);
                    step1done = true;
                }
            }
            else if(!step2done) {
                if (ex.getCurrentPosition() <= targetPossition) {
                    ex.setPower(0);
                    cs.setPosition(OPEN_POSSITION);
                    if(cs.getPosition() == OPEN_POSSITION){
                        targetPossition = targetPossition + 2 * LOCKED_POSITION;
                        ex.setPower(EXTRUSION_POWER / 3);
                        step2done = true;
                    }
                }
            }else{
                if(ex.getCurrentPosition() >= targetPossition){
                    ex.setPower(0);
                    step1done = false;
                    step2done = false;
                }
            }
        }
    }

    public void intakeManual(){
        //intake controls
        if(Math.abs(g2.right_stick_x) > DEAD_ZONE || Math.abs(g2.right_stick_y) > DEAD_ZONE){
            rx = .3 * g2.right_stick_x;
            ry = -g2.right_stick_y;
            il.setPower(ry + rx);
            ir.setPower(ry - rx);
        }else{
            il.setPower(0);
            ir.setPower(0);
        }
    }

    public void intakeAuto(double intakeSpeed){
        il.setPower(intakeSpeed);
        ir.setPower(intakeSpeed);
    }

    public void clampBuildPlate(boolean Clamp, boolean Unclamp){
        //clamping build plate controls
        if(Clamp){
            br.setPosition(BUILD_CLAPED_RIGHT);
            bl.setPosition(BUILD_CLAPED_LEFT);
        }else if(Unclamp){
            br.setPosition(BUILD_UNCLAPED_RIGHT);
            bl.setPosition(BUILD_UNCLAPED_LEFT);
        }
    }

    public void clampBlock(boolean Clamp, boolean Unclamp){
        //clmaping block controls
        if(Clamp){
            cs.setPosition(CLOSED_POSSITION);
        }else if(Unclamp){
            cs.setPosition(OPEN_POSSITION);
        }
    }

    public void blockHeight(boolean primed){
        if(primed) {
            if(releasedblockHeight){
                targetPossition = BLOCK_HEIGHT;
                ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ex.setPower(EXTRUSION_POWER / 3);
            }else if(targetPossition <= ex.getCurrentPosition()){
                ex.setPower(0);
            }
            releasedblockHeight = false;
        }else{
            releasedblockHeight = true;
        }
    }

    public void prime(boolean primed){
        if(primed) {
            if(releasedprime) {
                targetPossition = EXTRUSION_PRIME;
                ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ex.setPower(EXTRUSION_POWER / 3);
                rs.setPosition(SERVO_PRIME);
            }
            else if(targetPossition <= ex.getCurrentPosition()){
                ex.setPower(0);
            }
            releasedprime = false;
        }else{
            releasedprime = true;
        }
    }
    public void resetExtrusion(boolean reset, boolean bottom){
        if(reset) {
            rs.setPosition(MINIMUM_ROTATION);
            ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ex.setPower(-EXTRUSION_POWER);
        }
        if(bottom){
            ex.setPower(0);
            ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void extrudeToLevel(boolean extend){
        if(extend) {
            if(releasedextrudeToLevel) {
                targetPossition = level * LEVEL_HEIGHT + FIRST_LEVEL_HEIGHT;
                ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ex.setPower(EXTRUSION_POWER);
                rs.setPosition(MAX_ROTATION);
            }
            if(targetPossition < ex.getCurrentPosition()){
                ex.setPower(0);
            }
            releasedextrudeToLevel = false;
        }else{
            releasedextrudeToLevel = true;
        }
    }

    public void changeLevel(boolean levelUp, boolean levelDown){
        //controls for increasing level
        if(levelUp){
            if(releasedchangeLevel && level < 8) {
                level++;
                releasedchangeLevel = false;
            }
        }
        //controls for decreasing level
        else if(levelDown){
            if(releasedchangeLevel && level > 0) {
                level--;
                releasedchangeLevel = false;
            }
        }else{
            releasedchangeLevel = true;
        }
    }

    public String getLevel(){
        switch(level){
            case 0:
                return "   _____           \n" +
                        " /       \\  \n" +
                        "|  \\      | \n" +
                        "|    \\    | \n" +
                        "|      \\  | \n" +
                        " \\ _____ /       \n"
                        ;
            case 1:
                return "     __          \n" +
                        "   /   |        \n" +
                        "  /    |        \n" +
                        "       |        \n" +
                        "       |        \n" +
                        "  _____|__   \n"
                        ;
            case 2:
                return "     ____    \n" +
                        "    /    |   \n" +
                        "        /     \n" +
                        "      /        \n" +
                        "    /           \n" +
                        "   |______  \n"
                        ;
            case 3:
                return "   _______         \n" +
                        "  /      / \n" +
                        "        /   \n" +
                        "        \\   \n" +
                        "         \\ \n" +
                        "  \\_______\\     \n"
                        ;
            case 4:
                return "|       |\n" +
                        "|_______|      \n" +
                        "        |\n" +
                        "        |\n" +
                        "        |\n" +
                        "        |\n"
                        ;
            case 5:
                return " ______     \n" +
                        "/            \n" +
                        "\\____       \n" +
                        "     \\     \n" +
                        "      |      \n" +
                        "_____/       \n"
                        ;
            case 6:
                return "    ____        \n" +
                        "  /          \n" +
                        " /_____       \n" +
                        "|      \\    \n" +
                        "|       |    \n" +
                        " \\_____/\n"
                        ;
            case 7:
                return "_______\n" +
                        "       /\n" +
                        "      /\n" +
                        "    /\n" +
                        "   /\n" +
                        "  /\n"
                        ;
            case 8:
                return "   _____     \n" +
                        " /       \\    \n" +
                        "| _______ |    \n" +
                        "|         |    \n" +
                        "|         |      \n" +
                        " \\ _____ /      \n";
            default:
                return Integer.toString(level);
        }
    }


    public int getExtrusion(){
        ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return ex.getCurrentPosition();
    }

    public void stuff(){
        if(g1.left_stick_y != 0) {
            ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ex.setPower(-g1.left_stick_y);
        }else {
            ex.setPower(0);
        }
    }

}
