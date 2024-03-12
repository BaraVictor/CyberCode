////facut de Bara Victor 08.02.2024
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.bosch.BHI260IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.opencv.core.*;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.opencv.imgproc.Imgproc;
//import org.opencv.imgproc.Moments;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import java.util.ArrayList;
//import java.util.List;
//import com.arcrobotics.ftclib.controller.PIDController;
//
//@Config
//@Autonomous(name = "Autonomus", group = "Autonomous")
//public class Autonomus extends LinearOpMode {
//    private PIDController controller;
//    boolean usingPid=false;
//    public static double p = 0.05, i = 0.01, d = 0.001;
//    public static double f = 0.05;
//    public static int target = 0;
//    private final double ticks_in_degree = 288 / 180.0;
//    private DcMotorEx LeftArmMotor = null;
//    private DcMotorEx RightArmMotor = null;
//    private final int MAX_POSITION = 0;
//    private final int MIN_POSITION = -1000;
//    private DcMotor LeftMotor  = null;
//    private DcMotor RightMotor = null;
//    BHI260IMU imu;
//    double centi=17.4;
//    //ColorSensor senzorCuloare;
//    private Servo demoServo;
//    private Servo demoServoRight;
//    private Servo demoServoLeft;
//    double cX = 0;
//    double cY = 0;
//    double width = 0;
//    double open1 = 0.71;
//    double closed1 = 0.89;
//    double open2 = 0.75;
//    double closed2 = 0.54;
//
//    double a = 0.66;
//    double b = 0.84;
//    double y = 0.4;
//    double x = 0.305;
//    int caz = 1;
//
//    boolean objectDetected = false;
//
//    //pidf pt motoare sasiu
//
//    private static int FieldTarget = 0;
//
//   private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
//    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
//    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
//
//    // Calculate the distance using the formula
//    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
//    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels
//    private Orientation lastAngles = new Orientation();
//    private double currAngle = 0.0;
//
//    @Override
//    public void runOpMode() {
//        LeftMotor = (DcMotor)this.hardwareMap.dcMotor.get("LeftMotor");
//        RightMotor = (DcMotor)this.hardwareMap.dcMotor.get("RightMotor");
//        //senzorCuloare = hardwareMap.colorSensor.get("senzor_culoare");
//        LeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        //senzorCuloare.enableLed(true);
//        demoServo = hardwareMap.servo.get("servo");
//        demoServoRight = hardwareMap.servo.get("servo2");
//        demoServoLeft = hardwareMap.servo.get("servo3");
//
//        controller = new PIDController(p, i, d);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        LeftArmMotor = hardwareMap.get(DcMotorEx.class,"LeftArmMotor");
//        RightArmMotor = hardwareMap.get(DcMotorEx.class,"RightArmMotor");
//        RightArmMotor.setDirection(DcMotor.Direction.REVERSE);
//        LeftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LeftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        RightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        setAllPower(0);
//        LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        demoServoRight.setPosition(closed1);
//        demoServoLeft.setPosition(closed2);
//        demoServo.setPosition(b);
////        initOpenCV();
//        OpenCvCamera camera;
//        public void runOpMode() throws InterruptedException {
//            int cameraMonitorViewId = hardwareMap.appContext
//                    .getResources().getIdentifier("cameraMonitorViewId",
//                            "id", hardwareMap.appContext.getPackageName());
//            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
//            RedTeamProp detector = new RedTeamProp(telemetry);
//            camera.setPipeline(detector);
//            camera.openCameraDevice();
//            camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
//            FtcDashboard dashboard = FtcDashboard.getInstance();
//            telemetry = new MultipleTelemetry(telemetry , dashboard.getTelemetry());
//            FtcDashboard.getInstance().startCameraStream(camera, 30);
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
//        imu = hardwareMap.get(BHI260IMU.class,"imu");
//        imu.initialize();
//        waitForStart();
////            if (cX > 30 && cX < 330)
////                caz = 2;
////            else if (cX > 450 && cX < 570)
////                caz = 3;
////            else caz = 1;
//            switch (detector.getLocation()) {
//                case LEFT:
//                    telemetry.addLine("stanga");
//                    break;
//                case RIGHT:
//                    telemetry.addLine("dreapta");
//                    break;
//                case MID:
//                    telemetry.addLine("mid");
//                    break;
//            }
//
//            controller.setPID(p, i, d);
//            int armPos = LeftArmMotor.getCurrentPosition();
//            double pid = controller.calculate(armPos, target);
//            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
//            double power = pid + ff;
//
//            //pas(1...16)
//
//
//            telemetry.addData("Arm Position:",armPos);
//            telemetry.addData("target ",target);
//            telemetry.update();
//            controlHubCam.stopStreaming();
//        }
//    }
//    public void setAllPower(double p ){setMotorPower(p,p);}
//    public void resetEncoders(){
//        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//    public void setMotorPower(double lF, double rF){
//        LeftMotor.setPower(lF);
//        LeftMotor.setPower(rF);
//    }
//    public void resetAngle(){
//        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZXY,AngleUnit.DEGREES);
//        currAngle = 0;
//    }
//    public double getAngle() {
//
//        // Get current orientation
//        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
//
//        // Change in angle = current angle - previous angle
//        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
//
//        // Gyro only ranges from -179 to 180
//        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
//        if (deltaAngle < -180) {
//            deltaAngle += 360;
//        } else if (deltaAngle > 180) {
//            deltaAngle -= 360;
//        }
//
//        // Add change in angle to current angle to get current angle
//        currAngle += deltaAngle;
//        lastAngles = orientation;
//        telemetry.addData("gyro", orientation.firstAngle);
//        return currAngle;
//    }
//    void goToRight(double targetRight){
//        resetEncoders();
//        targetRight*=centi;
//        double error2 = targetRight;
//        while (opModeIsActive() && Math.abs(error2) > 10) {
//            double motorPower = (error2 < 0 ? -0.40 : 0.40);
//            setMotorPower(motorPower, (motorPower-0.03 ));
//            error2 = targetRight - RightMotor.getCurrentPosition();
//            telemetry.addData("error", error2);
//            telemetry.addData("position",RightMotor.getCurrentPosition());
//            telemetry.update();
//        }
//        setAllPower(0);
//    }
//
//    void goToLeft(double targetLeft){
//        resetEncoders();
//        targetLeft*=centi;
//        double error3 = targetLeft;
//        while (opModeIsActive() && Math.abs(error3) > 10) {
//            double motorPower = (error3 < 0 ? -0.40 : 0.40);
//            setMotorPower(motorPower, (motorPower-0.03 ));
//            error3 = targetLeft - LeftMotor.getCurrentPosition();
//            telemetry.addData("error", error3);
//            telemetry.addData("position",LeftMotor.getCurrentPosition());
//            telemetry.update();
//        }
//        setAllPower(0);
//    }
////    private void initOpenCV() {
////
////        // Create an instance of the camera
////        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
////                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
////
////        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
////        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
////                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
////
////        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());
////
////        controlHubCam.openCameraDevice();
////        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
////    }
////
////    class YellowBlobDetectionPipeline extends OpenCvPipeline {
////        @Override
////        public Mat processFrame(Mat input) {
////            // Preprocess the frame to detect yellow regions
////            Mat yellowMask = preprocessFrame(input);
////
////            // Find contours of the detected yellow regions
////            List<MatOfPoint> contours = new ArrayList<>();
////            Mat hierarchy = new Mat();
////            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
////
////            // Find the largest yellow contour (blob)
////            MatOfPoint largestContour = findLargestContour(contours);
////
////            if (largestContour != null) {
////                // Draw a red outline around the largest detected object
////                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
////                // Calculate the width of the bounding box
////                width = calculateWidth(largestContour);
////
////                // Display the width next to the label
////                String widthLabel = "Width: " + (int) width + " pixels";
////                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
////                //Display the Distance
////                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
////                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
////                // Calculate the centroid of the largest contour
////                Moments moments = Imgproc.moments(largestContour);
////                cX = moments.get_m10() / moments.get_m00();
////                cY = moments.get_m01() / moments.get_m00();
////
////                // Draw a dot at the centroid
////                String label = "(" + (int) cX + ", " + (int) cY + ")";
////                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
////                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
////
////            }
////
////            return input;
////        }
////
////        private Mat preprocessFrame(Mat frame) {
////            Mat hsvFrame = new Mat();
////            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);
////
////            Scalar lowerYellow = new Scalar(110, 100, 0);
////            Scalar upperYellow = new Scalar(135, 255, 255);
////
////
////            Mat yellowMask = new Mat();
////            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
////
////            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
////            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
////            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);
////
////            return yellowMask;
////        }
////
////        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
////            double maxArea = 0;
////            MatOfPoint largestContour = null;
////
////            for (MatOfPoint contour : contours) {
////                double area = Imgproc.contourArea(contour);
////                if (area > maxArea) {
////                    maxArea = area;
////                    largestContour = contour;
////                }
////            }
////            return largestContour;
////        }
////
////        private double calculateWidth(MatOfPoint contour) {
////            Rect boundingRect = Imgproc.boundingRect(contour);
////            return boundingRect.width;
////        }
////    }
////
////    private static double getDistance(double width) {
////        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
////        return distance;
////    }
//
//    private void Pas1 () { //mergi in fata pana la spike
//        target=-20;
//        demoServo.setPosition(a);
//        Close();
//        sleep(500);
//    }
//
//    private void Pas2 (){
//        Forward(64);
//    }
//
//    private void Pas3 () { //roteste pt caz
//        if(caz==1){
//            turnRight(90);
//        }
//        else if(caz==2){
//            Forward(64);
//        }
//        else if(caz==3){
//            turnLeft(90);
//        }
//    }
//
//    private void Pas4 () { //lasa pixelul mov jos
//        target=0;
//        demoServo.setPosition(a);
//        demoServoRight.setPosition(open1);
//        sleep(1000);
//    }
//
//    private void Pas5 () { //ridica bratul si pune gheara in poz b
//        target=-20;
//        demoServo.setPosition(b);
//        Close();
//        sleep(1000);
//    }
//
//    private void Pas6 () {   //roteste sasiul sa fie cu spatele la tabla
//        if(caz==1){
//            turnLeft(180);
//        }
//        else if(caz==2){
//            turnLeft(90);
//        }
//    }
//
//    private void Pas7 () {   //se duce in spate pana la poz x
//        Forward(65);
//    }
//
//    private void Pas8 () {  //rotire paralel cu tabla >.<
//        turnRight(90);
//    }
//
//    private void Pas9 () {  //aliniere caz tabla
//        if(caz==1){
//            Forward(10);
//        }
//        else if(caz==3){
//            Backward(10);
//        }
//    }
//
//    private void Pas10 () {  //rotire perpendicular cu tabla >.<
//        turnLeft(90);
//    }
//
//    private void Pas11 () { //se duce cu bratul la poz x si lasa pixelul
//        target=-700;
//        demoServo.setPosition(x);
//        sleep(500);
//        Open();
//        sleep(50);
//    }
//
//    private void Pas12 () { //se duce cu bratul la poz b si inchide inapoi ghiara
//        target=-20;
//        demoServo.setPosition(b);
//        sleep(500);
//        Close();
//        sleep(50);
//    }
//
//    private void Pas13 () { //se roteste sasiulsa fie paralele cu tabla
//        turnRight(90);
//    }
//
//    private void Pas14 () { //se duce in spate si se alinaza pt parcare
//        if(caz==1){
//            Forward(66);
//        }
//        else if(caz==2){
//            Forward(76);
//        }
//        else if(caz==3){
//            Forward(86);
//        }
//    }
//
//    private void Pas15 () { //se sa se aliniaze cu parcarea
//        turnLeft(90);
//    }
//
//    private void Pas16 () { //se da in spate pt a fi parcat
//        Backward(66);
//    }
//
//    private void Pas17 () {
//        target=0;
//        demoServo.setPosition(a);
//        Open();
//        sleep(500);
//    }
//
//
//    public void turnRight(double Grade){
//        goToRight(Grade*15);
//        goToLeft(Grade*(-15));
//    }
//
//    public void turnLeft(double Grade){
//        goToLeft(Grade*15);
//        goToRight(Grade*(-15));
//    }
//
//    public void Forward(double Centimetri){
//        goToRight(Centimetri);
//        goToLeft(Centimetri);
//    }
//
//    public void Backward(double Centimetri){
//        goToRight(-Centimetri);
//        goToLeft(-Centimetri);
//    }
//
//    private void Open () {
//        demoServoRight.setPosition(open1);
//        demoServoLeft.setPosition(open2);
//    }
//
//    private void Close () {
//        demoServoRight.setPosition(closed1);
//        demoServoLeft.setPosition(closed2);
//    }
//
//    private void Stop() {
//        setAllPower(0);
//        sleep(500);
//    }
//
//    private void MaxStop() {
//        setAllPower(0);
//        sleep(2000);
//    }
//
//    private void SuperMaxStop() {
//        setAllPower(0);
//        sleep(5000);
//    }
//
//    private void ExtraSuperMaxStop() {
//        setAllPower(0);
//        sleep(10000);
//    }
//}