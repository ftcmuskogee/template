package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/** set it into the telop folder and the name that will show up**/
@TeleOp(name="telelop_without_rr")

public class telelop_without_rr extends LinearOpMode {
    // names motors and sets motors to type null
    public DcMotor Frontright = null;
    public DcMotor Backright = null;
    public DcMotor Backleft = null;
    public DcMotor Frontleft = null;
    public DcMotor LinAct = null;
    public Servo ClawL = null;
    public Servo ClawR = null;
    public Servo Wrist = null;
    public CRServo LinAngle = null;
    public DcMotor ArmAngle = null;
    public DcMotor Armextend = null;

    //plane launch
    public Servo Plane = null;
    // sets hardware map to null and names it


    //calls hardware map
    HardwareMap hardwaremap = null;

    @Override
    public void runOpMode() {
        double speed;
        double extend;
        Frontright = hardwareMap.get(DcMotor.class, "RF");
        Backright = hardwareMap.get(DcMotor.class, "RB");
        Backleft = hardwareMap.get(DcMotor.class, "LB");
        Frontleft = hardwareMap.get(DcMotor.class, "LF");
        LinAct = hardwareMap.get(DcMotor.class, "ACT");
        ClawL = hardwareMap.get(Servo.class, "CL");
        ClawR = hardwareMap.get(Servo.class, "CR");
        Armextend = hardwareMap.get(DcMotor.class, "ARM");
        Wrist = hardwareMap.get(Servo.class, "W");
        LinAngle = hardwareMap.get(CRServo.class, "LA");
        ArmAngle = hardwareMap.get(DcMotor.class, "AA");
        Plane = hardwareMap.get(Servo.class, "P");

        Frontleft.setDirection(DcMotor.Direction.REVERSE);
        Backleft.setDirection(DcMotor.Direction.REVERSE);

        Frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Armextend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Armextend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Armextend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LinAct.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinAct.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinAct.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //calls from samplemecanumdrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //sets motors to run without encoders
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //initializes hardware map
        //robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            /** not tested before may or may not work**/
// if power1 is forward/backward
            double Power1 = gamepad2.right_stick_y;
            //if power 2 is turning
            double Power2 = gamepad2.right_stick_x;
            // could set this up for strafing -> 1 left and 1 right motor reversede below
            double Power3 = gamepad2.left_stick_y;

             //running at full power (no constraints of speed)
            Frontleft.setPower(Power1);
            Frontright.setPower(Power1);
            Backleft.setPower(Power1);
            Backright.setPower(Power1);

            Frontleft.setPower(-Power2);
            Frontright.setPower(Power2);
            Backleft.setPower(-Power2);
            Backright.setPower(Power2);

            /** for driving with buttons....also not tested**/
            //full power -> EX) 0.5 = 50% power
            //a is forward
            if (gamepad1.a){
                Frontleft.setPower(1);
                Frontright.setPower(1);
                Backleft.setPower(1);
                Backright.setPower(1);
            }
            //back
            else if (gamepad1.b) {
                Frontleft.setPower(-1);
                Frontright.setPower(-1);
                Backleft.setPower(-1);
                Backright.setPower(-1);
            }
            //right
            else if (gamepad1.x) {
                Frontleft.setPower(1);
                Frontright.setPower(-1);
                Backleft.setPower(1);
                Backright.setPower(-1);
            }
            //strafe right ... i think
            else if (gamepad1.y) {
                Frontleft.setPower(1);
                Frontright.setPower(-1);
                Backleft.setPower(-1);
                Backright.setPower(1);
            }
            //.....then flip for left, and strafing set up




            drive.update();
            //drive controller

            /** remember to zero servos**/

            //down
            if (gamepad1.right_bumper) {
                LinAct.setPower(-.2);
            }
            //up
            else if (gamepad1.left_bumper) {
                LinAct.setPower(.2);
            } else {
                LinAct.setPower(0);
            }
//continuous rotatioin servos run on set power not set position
            if (gamepad1.y) {
                LinAngle.setPower(-10);
            }

            else if (gamepad1.x) {
                LinAngle.setPower(10);
            }
            else
            {
                LinAngle.setPower(0);
            }


            //arm controller


            // Makes variables Power1 and Power2 to their respective joystick
            double Power45 = gamepad2.right_stick_y;
            double Power13 = gamepad2.left_stick_y;
            speed = -0.1; //.02
            extend = -0.2;
            // sets the power for the lifts

            Armextend.setPower(Power45 * extend);
            ArmAngle.setPower(Power13 * speed);


//5 turn so change
            //on ground
            if (gamepad2.right_bumper){
                Wrist.setPosition(1);
                //.65
            }
            // backwards scoring
            if (gamepad2.left_bumper){
                Wrist.setPosition(.9);
                //.75  0 needs to be redifined
            }

            //SERVO NUMBERS NEED TO BE OPPOSITE -> for mirrored claws

            if (gamepad2.left_trigger > 0.1) {
                ClawL.setPosition(.52);
            }
            else if (gamepad2.right_trigger > 0.1){
                ClawR.setPosition(0);
            }
            else{
                ClawL.setPosition(.48);
                ClawR.setPosition(.15);
            }

            if (gamepad2.x) {
                Plane.setPosition(0);
            }

            //adds data to the driver hub that tells you the coordinates of where the robot is on the field
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("a", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

        }
    }
}
