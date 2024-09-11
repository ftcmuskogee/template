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
@Autonomous(name = "auto_with_rr", group = "Autonomous Main")
public class auto_with_rr extends LinearOpMode {

    /** vision variables are called first (for times sake theyre not in this document)**/
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */

    @Override
    public void runOpMode() {
        // the current range set by lower and upper is the full range
        // HSV takes the form: (HUE, SATURATION, VALUE)
        // which means to select our colour, only need to change HUE
        // the domains are: ([0, 180], [0, 255], [0, 255])
        // this is tuned to detect red, so you will need to experiment to fine tune it for your robot
        // and experiment to fine tune it for blue
        Scalar lower = new Scalar(110, 50, 50); // the lower hsv threshold for your detection
        Scalar upper = new Scalar(130, 255, 255); // the upper hsv threshold for your detection
        double minArea = 90; // the minimum area for the detection to consider for your prop
/*** up min area value incase the distance is limited - cause it sees it up close but not far ***/
        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(colourMassDetectionProcessor)
                .build();

        // you may also want to take a look at some of the examples for instructions on
        // how to have a switchable camera (switch back and forth between two cameras)
        // or how to manually edit the exposure and gain, to account for different lighting conditions
        // these may be extra features for you to work on to ensure that your robot performs
        // consistently, even in different environments

        /**
         * User-defined init_loop method
         * <p>
         * This method will be called repeatedly during the period between when
         * the init button is pressed and when the play button is pressed (or the
         * OpMode is stopped).
         * <p>
         * This method is optional. By default, this method takes no action.
         */

        /** hardware map set up**/
        hardwaremap robot = new hardwaremap();
        robot.init(hardwareMap);

        /**initialization loop for stuff to appear on driver hub **/
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
            telemetry.update();
        }


        /**
         * User-defined start method
         * <p>
         * This method will be called once, when the play button is pressed.
         * <p>
         * This method is optional. By default, this method takes no action.
         * <p>
         * Example usage: Starting another thread.
         */
        // shuts down the camera once the match starts, we dont need to look any more
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        // gets the recorded prop position
        ColourMassDetectionProcessor.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == ColourMassDetectionProcessor.PropPositions.UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessor.PropPositions.MIDDLE;
        }

        /** rr set up**/
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        /** starting point on the feild -> (35,60) is an example quardinate in quad 1,
         * 270 is the compass angle you are facing
          */
        Pose2d startPose = new Pose2d(35, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
/**trajectories, they are usually paired depending on your code, they can rely on start position,
 *  a unique quardinate, or the end of another trajectory**/
        //middle forward
        TrajectorySequence A = drive.trajectorySequenceBuilder(startPose)
                //runs from start pose to the point listed
                // the next numbers are the constraints set up in roadrunner,
                // you can use different numbers if you want it to go faster or slower
                .lineToConstantHeading(new Vector2d(35, 34), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();
        //back,strafe
        TrajectorySequence B = drive.trajectorySequenceBuilder(A.end())
                .waitSeconds(1)
                // descriptions of movements in learnrr.com
                .lineToConstantHeading(new Vector2d(35, 37), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(70, 25, Math.toRadians(0)))
                .build();


        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (recordedPropPosition) {
            // code is dependent on camera
            // can set up an else
            case LEFT:
                // example auto
                //wrist (from hardwaremap)
                robot.W(1);
                sleep(1000);
                // arm movement (from hmap)
                robot.UP(.03);
                sleep(100);
                // movement through rr
                drive.followTrajectorySequence(A);
                robot.W(1);
                //open right claw
                robot.CR(0);
                sleep(3000);
                robot.Aoff();
                drive.followTrajectorySequence(B);
                break;

            case MIDDLE:
                break;

            case RIGHT:
                break;

        }
        // camera set up tings
        while (opModeIsActive()) {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(5);
        }
        while (!isStopRequested())
        {
            colourMassDetectionProcessor.close();
            visionPortal.close();
        }
    }
}