package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="TeamPropAuto", group="C")
public class TeamPropAuto extends LinearOpMode {
    OpenCvCamera camera;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
        RedTeamProp detector = new RedTeamProp(telemetry);
        camera.setPipeline(detector);
        camera.openCameraDevice();
        camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry , dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(camera, 30);

        waitForStart();
        while(opModeIsActive()) {
            switch (detector.getLocation()) {
                case LEFT:
                    telemetry.addLine("stanga");
                    break;
                case RIGHT:
                    telemetry.addLine("dreapta");
                    break;
                case MID:
                    telemetry.addLine("mid");
                    break;
            }
            telemetry.update();
        }
        camera.stopStreaming();
    }
}