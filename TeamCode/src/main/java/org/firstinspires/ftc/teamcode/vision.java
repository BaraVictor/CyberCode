package org.firstinspires.ftc.teamcode;

import android.util.Size;
import org.firstinspires.ftc.teamcode.movement;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class vision extends LinearOpMode {
    movement movement = new movement();
    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessors(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .setCameraResolution(new Size(640, 480))
                .build();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("X", tag.ftcPose.x);
                telemetry.addData("Y", tag.ftcPose.y);
                telemetry.addData("Z", tag.ftcPose.z);
                if(tag.ftcPose.x<1)
                    movement.move_r();
                if(tag.ftcPose.x>1)
                    movement.move_l();
                if(tag.ftcPose.z>5)
                    movement.move_f();
            }
            telemetry.update();
        }
    }
}