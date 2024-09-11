package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

/** groups it in auto folder**/
@Autonomous(name = "auto_with_rr....and camera", group = "Autonomous Main")
public class auto_without_rr extends LinearOpMode {
    //THIS HAS NO CAMERA
    @Override
    public void runOpMode() {
        /** hardware map set up**/
        hardwaremap robot = new hardwaremap();
        robot.init(hardwareMap);

        
    }
}