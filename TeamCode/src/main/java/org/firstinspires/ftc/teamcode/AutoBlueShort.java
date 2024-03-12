package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous(name="AutoBlueShort", group="A")
public class AutoBlueShort extends LinearOpMode {
    OpenCvCamera camera;
    public DcMotor encoderRight;
    public DcMotor encoderLeft;
    BHI260IMU imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction, rotation;
    boolean                 aButton, bButton, touched;
    PIDController           pidRotate, pidDrive;
    double cpoz;
    double centi = 2.63;

    //

    private com.arcrobotics.ftclib.controller.PIDController controller;
    boolean usingPid=false;
    public static double p = 0.05, i = 0.01, d = 0.001;
    public static double f = 0.05;
    public static int target = 0;
    private final double ticks_in_degree = 288 / 180.0;
    private DcMotorEx LeftArmMotor = null;
    private DcMotorEx RightArmMotor = null;
    private final int MAX_POSITION = 0;
    private final int MIN_POSITION = -1000;
    private DcMotor LeftMotor  = null;
    private DcMotor RightMotor = null;
    //ColorSensor senzorCuloare;
    private Servo demoServo;
    private Servo demoServoRight;
    private Servo demoServoLeft;
    double open1 = 0.71;
    double closed1 = 0.91;
    double open2 = 0.75;
    double closed2 = 0.51;

    double a = 0.66;
    double b = 1.0;
    double y = 0.4;
    double x = 0.305;
    int caz = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
        BlueTeamProp detector = new BlueTeamProp(telemetry);
        camera.setPipeline(detector);
        camera.openCameraDevice();
        camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry , dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(camera, 30);
        LeftMotor = (DcMotor)this.hardwareMap.dcMotor.get("LeftMotor");
        RightMotor = (DcMotor)this.hardwareMap.dcMotor.get("RightMotor");
        LeftMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize();

        demoServo = hardwareMap.servo.get("servo");
        demoServoRight = hardwareMap.servo.get("servo2");
        demoServoLeft = hardwareMap.servo.get("servo3");
        LeftArmMotor = hardwareMap.get(DcMotorEx.class,"LeftArmMotor");
        RightArmMotor = hardwareMap.get(DcMotorEx.class,"RightArmMotor");
        RightArmMotor.setDirection(DcMotor.Direction.REVERSE);

        LeftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setAllPower(0);

        demoServoRight.setPosition(closed1);
        demoServoLeft.setPosition(closed2);
        target=-20;
        LeftArmMotor.setTargetPosition(target);
        RightArmMotor.setTargetPosition(target);
        LeftArmMotor.setPower(1.0);
        RightArmMotor.setPower(1.0);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        demoServo.setPosition(b);
        Close();
        pidRotate = new PIDController(.003, .00003, 0);
        pidDrive = new PIDController(.05, 0, 0);
        encoderLeft = LeftMotor;
        encoderRight = RightMotor;
        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        waitForStart();
        Pas0();
        MaxStop();
        switch (detector.getLocation()) {
            case LEFT:
                telemetry.addLine("stanga");
                caz = 1;
                break;
            case RIGHT:
                telemetry.addLine("dreapta");
                caz = 3;
                break;
            case NOT_FOUND:
                break;
            case MID: {
                telemetry.addLine("mid");
                caz = 2;
                break;
            }
        }
        telemetry.update();
        Pas1();
        camera.stopStreaming();
        Pas2();
        Pas3();
        Pas4();
        Pas5();
        Pas6();
        Pas7();
        Pas8();
        Pas9();
        Pas10();
        Pas11();
        Pas12();
        Pas13();
        Pas14();
        Pas15();
        Pas16();
        Pas17();
        Pas18();
    }
    private void resetAngle()
    {
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    private void rotate(double degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                LeftMotor.setPower(power*1.5);
                RightMotor.setPower(-power*1.5);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                LeftMotor.setPower(-power*1.5);
                RightMotor.setPower(power*1.5);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                LeftMotor.setPower(-power*1.5);
                RightMotor.setPower(power*1.5);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        RightMotor.setPower(0);
        LeftMotor.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
    public void resetEncoders(){
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setMotorPower(double lF, double rF){
        LeftMotor.setPower(lF);
        RightMotor.setPower(rF);
    }
    private void goTo(double target){
        resetEncoders();
        target*=centi;
        double error2 = target;
        while(opModeIsActive() && Math.abs(error2)>20){
            cpoz =(int)(((double)encoderRight.getCurrentPosition()/8192.0)*28.0);
            double motorPower =(error2<0? -0.50 : 0.50);
            setMotorPower(motorPower ,motorPower);
            error2 = target -cpoz;
            telemetry.addData("cpoz",cpoz);
            telemetry.addData("target",error2);
            telemetry.update();
        }
    }
    public void setAllPower(double p ){setMotorPower(p,p);}

    private void Pas0(){
        target=-20;
        LeftArmMotor.setTargetPosition(target);
        RightArmMotor.setTargetPosition(target);
        LeftArmMotor.setPower(1.0);
        RightArmMotor.setPower(1.0);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        demoServo.setPosition(a);
        Close();
        Stop();
    }

    private void Pas1 (){
        Forward(61);
    }

    private void Pas2 () { //roteste pt caz
        if(caz==1){
            turnRight(55);
        }
        else if(caz==2){
            Forward(20);
            Stop();
            Backward(10);
        }
        else if(caz==3){
            turnLeft(88);
        }
    }

    private void Pas3(){
        if (caz==2 || caz==1) {
            demoServoLeft.setPosition(open2);
            sleep(100);
            Stop();
            demoServo.setPosition(1.0);
        }
        if(caz==3){
            demoServoLeft.setPosition(open2);
            sleep(100);
            Stop();
            demoServo.setPosition(1.0);
        }
    }

    private void Pas4(){
        if(caz==1){
            turnLeft(90);
        }
        else if(caz==2){
            turnLeft(90);
            Forward(25);
            turnRight(90);
        }
        else if(caz==3){
            turnRight(88);
        }
    }

    private void Pas5(){
        Forward(57);
    }

    private void Pas6(){
        turnLeft(87);
    }

    private void Pas7(){
        if(caz==2){
            Backward(195);
        }
        else{
            Backward(172);
        }
    }

    private void Pas8(){
        turnRight(84);
    }

    private void Pas9(){
        if(caz==1){
//            target=-700;
//            LeftArmMotor.setTargetPosition(target);
//            RightArmMotor.setTargetPosition(target);
//            LeftArmMotor.setPower(1.0);
//            RightArmMotor.setPower(1.0);
//            LeftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            RightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            MaxStop();
//            demoServo.setPosition(x);
            Backward(45);
        }
        if(caz==2){
//            target=-700;
//            LeftArmMotor.setTargetPosition(target);
//            RightArmMotor.setTargetPosition(target);
//            LeftArmMotor.setPower(1.0);
//            RightArmMotor.setPower(1.0);
//            LeftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            RightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            Stop();
//            demoServo.setPosition(x);
            Backward(45);
        }
        if(caz==3){
//            target=-700;
//            LeftArmMotor.setTargetPosition(target);
//            RightArmMotor.setTargetPosition(target);
//            LeftArmMotor.setPower(1.0);
//            RightArmMotor.setPower(1.0);
//            LeftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            RightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            Stop();
//            demoServo.setPosition(x);
            Backward(45);
        }
    }

    private void Pas10(){
            turnLeft(100);
    }

    private void Pas11(){
        target=-700;
        LeftArmMotor.setTargetPosition(target);
        RightArmMotor.setTargetPosition(target);
        LeftArmMotor.setPower(1.0);
        RightArmMotor.setPower(1.0);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MaxStop();
        demoServo.setPosition(x);
        MaxStop();
        Backward(30);
        Stop();
    }

    private void Pas12(){
        Open();
    }

    private void Pas13(){
        Forward(15);
        target=-40;
        LeftArmMotor.setTargetPosition(target);
        RightArmMotor.setTargetPosition(target);
        LeftArmMotor.setPower(1.0);
        RightArmMotor.setPower(1.0);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        demoServo.setPosition(b);
        Close();
    }

    private void Pas14(){
        turnRight(90);
    }

    private void Pas15(){
            Forward(48);
    }

    private void Pas16(){
        turnLeft(90);
    }

    private void Pas17(){
        Backward(15);
    }

    private void Pas18() {
        target=0;
        LeftArmMotor.setTargetPosition(target);
        RightArmMotor.setTargetPosition(target);
        LeftArmMotor.setPower(1.0);
        RightArmMotor.setPower(1.0);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        demoServo.setPosition(a);
        Open();
        sleep(250);
    }

    private void BratPozX () { //se duce cu bratul la poz x si lasa pixelul
        target=-700;
        LeftArmMotor.setTargetPosition(target);
        RightArmMotor.setTargetPosition(target);
        LeftArmMotor.setPower(1.0);
        RightArmMotor.setPower(1.0);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        demoServo.setPosition(1.0);
        Close();
        setAllPower(0);
        sleep(2000);
    }

    private void BratPozB () { //se duce cu bratul la poz b si inchide inapoi ghiara
        target=-20;
        LeftArmMotor.setTargetPosition(target);
        RightArmMotor.setTargetPosition(target);
        LeftArmMotor.setPower(1.0);
        RightArmMotor.setPower(1.0);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        demoServo.setPosition(b);
        setAllPower(0);
        sleep(2000);
        Close();
        sleep(50);
    }


    public void turnRight(double Grade){
        rotate(Grade, power);
    }

    public void turnLeft(double Grade){
        rotate(-Grade, power);
    }

    public void Forward(double Centimetri){
        goTo(Centimetri);
    }

    public void Backward(double Centimetri){
        goTo(-Centimetri);
    }

    private void Open () {
        demoServoRight.setPosition(open1);
        demoServoLeft.setPosition(open2);
        sleep(100);
    }

    private void Close () {
        demoServoRight.setPosition(closed1);
        demoServoLeft.setPosition(closed2);
        sleep(100);
    }

    private void Stop() {
        setAllPower(0);
        sleep(500);
    }

    private void BigStop() {
        setAllPower(0);
        sleep(850);
    }

    private void MaxStop() {
        setAllPower(0);
        sleep(2000);
    }

    private void SuperMaxStop() {
        setAllPower(0);
        sleep(5000);
    }

    private void ExtraSuperMaxStop() {
        setAllPower(0);
        sleep(10000);
    }
}
