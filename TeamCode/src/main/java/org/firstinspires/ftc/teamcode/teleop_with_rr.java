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
@TeleOp(name="teleop_with_rr")

public class teleop_with_rr extends LinearOpMode {
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

            //sets all 4 base motors to the left and right joysticks on gamepad 1
            //uses the variables from SampleMecanumDrive to adjust motors
            //left stick in the y direction is for going forward and backward at 80% power
            //left stick in the x direction is for strafing left and right at 80% power
            //right stick in the x direction is for turning left and right at 80% power
            //was x: y
            //was y: x
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 1,
                            -gamepad1.left_stick_x * 1,
                            -gamepad1.right_stick_x * 0.8
                    )
            );

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
            double Power1 = gamepad2.right_stick_y;
            double Power2 = gamepad2.left_stick_y;
            speed = -0.1; //.02
            extend = -0.2;
            // sets the power for the lifts

            Armextend.setPower(Power1 * extend);
            ArmAngle.setPower(Power2 * speed);


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
