//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="vision", group="B")
public class vision extends LinearOpMode {
    public vision() {
    }

    public void runOpMode() throws InterruptedException {
        AprilTagProcessor tagProcessor = (new AprilTagProcessor.Builder()).setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).build();
        VisionPortal visionPortal = (new VisionPortal.Builder()).addProcessors(new VisionProcessor[]{tagProcessor}).setCamera((CameraName)this.hardwareMap.get(WebcamName.class, "Webcam1")).setCameraResolution(new Size(640, 480)).build();
        this.waitForStart();

        for(; !this.isStopRequested() && this.opModeIsActive(); this.telemetry.update()) {
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = (AprilTagDetection)tagProcessor.getDetections().get(0);
                this.telemetry.addData("X", tag.ftcPose.x);
                this.telemetry.addData("Y", tag.ftcPose.y);
                this.telemetry.addData("Z", tag.ftcPose.z);
                this.telemetry.addData("ROLL", tag.ftcPose.roll);
                this.telemetry.addData("PITCH", tag.ftcPose.pitch);
                this.telemetry.addData("YAW", tag.ftcPose.yaw);
            }
        }

    }
}
