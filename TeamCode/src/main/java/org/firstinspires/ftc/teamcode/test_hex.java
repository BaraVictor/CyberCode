package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "hex")
public class test_hex extends OpMode {
    private DcMotor armLeft;
    private DcMotor armRight;
    DcMotor motor2;
    DcMotor motor1;
    private Servo demoServo;
    private Servo demoServoRight;
    private Servo demoServoLeft;
    private final int armScorePosition = -500;
    public int armHomePosition = 2;
    private final int armIntakePosition = 10;

    private static final double MAX_POSITION = 0;

    private static final double MIN_POSITION = -1000;
    private static final double MAX_POSITION_SERVO = 0.4;
    private static final double MIN_POSITION_SERVO = 0.0;
    private static final double MAX_POSITION_SERVO_WRIST = 1;
    private static final double MIN_POSITION_SERVO_WRIST = -0.5;
    private ElapsedTime runtime = new ElapsedTime();
 // turn that frown upside down :D
    @Override
    public void init() {

        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        armLeft.setDirection(DcMotor.Direction.FORWARD);
        armRight.setDirection(DcMotor.Direction.REVERSE);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setPower(0.0);
        armRight.setPower(0.0);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armHomePosition = armLeft.getCurrentPosition();

        demoServo = hardwareMap.servo.get("servo");
        demoServoRight = hardwareMap.servo.get("servo2");
        demoServoLeft = hardwareMap.servo.get("servo3");


        motor1 = (DcMotor) this.hardwareMap.dcMotor.get("motor1");
        motor2 = (DcMotor) this.hardwareMap.dcMotor.get("motor2");

    }

    @Override
    public void start() {
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("ticks: ", armLeft.getCurrentPosition());
        demoServoRight = hardwareMap.servo.get("servo2");          // 2 servos,
        demoServoLeft = hardwareMap.servo.get("servo3");

        double leftStickX = (double) this.gamepad2.left_stick_y;
        double leftStickY = (double) this.gamepad2.left_stick_x;
        double motor2Power = leftStickY + leftStickX;
        double motor1Power = leftStickY - leftStickX;
        motor1.setPower(-motor1Power / 2);
        motor2.setPower(-motor2Power / 2);

        if (gamepad1.right_trigger > 0.1) {  //strange
            demoServoRight.setPosition(demoServoRight.getPosition() - 0.2);
        }
        // Check if LT (Left Trigger) is pressed to set the servo to min position
        else if (gamepad1.right_bumper != false) {     //restranghe
            demoServoRight.setPosition(demoServoRight.getPosition() + 0.2);
        }

        if (gamepad1.left_trigger > 0.1) {  //strange
            demoServoLeft.setPosition(demoServoLeft.getPosition() + 0.2);
        }
        // Check if LT (Left Trigger) is pressed to set the servo to min position
        else if (gamepad1.left_bumper != false) {     //restranghe
            demoServoLeft.setPosition(demoServoLeft.getPosition() - 0.2);
        }

        if (gamepad1.right_stick_y > 0) {
            if (demoServo.getPosition() < MAX_POSITION_SERVO_WRIST) {
                demoServo.setPosition(demoServo.getPosition() + 0.02);
            }
        }
        // Check if LT (Left Trigger) is pressed to set the servo to min position
        else if (gamepad1.right_stick_y < 0) {
            if (demoServo.getPosition() > MIN_POSITION_SERVO_WRIST) {
                demoServo.setPosition(demoServo.getPosition() - 0.02);
            }
        }

        if (gamepad1.left_stick_y > 0 && armLeft.getCurrentPosition() <= MAX_POSITION) {
            armLeft.setPower(-1); // Adjust power based on your needs
            armRight.setPower(-1); // Adjust power based on your needs
        }
        // Gamepad button Y for moving the arm down
        else if (gamepad1.left_stick_y < 0 && armLeft.getCurrentPosition() >= MIN_POSITION) {
            armLeft.setPower(1); // Adjust power based on your needs
            armRight.setPower(1); // Adjust power based on your needs
        }
        // Stop the arm if no buttons are pressed
        else {
            armLeft.setPower(0);
            armRight.setPower(0);
        }

        if (gamepad1.y) {
            armLeft.setTargetPosition(armScorePosition);
            armRight.setTargetPosition(armScorePosition);
            armLeft.setPower(1.0 / 4);
            armRight.setPower(1.0 / 4);
            armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad1.a) {
            armLeft.setTargetPosition(armHomePosition);
            armRight.setTargetPosition(armHomePosition);
            armLeft.setPower(1.0 / 4);
            armRight.setPower(1.0 / 4);
            armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (gamepad1.b) {
            armLeft.setTargetPosition(armIntakePosition);
            armRight.setTargetPosition(armIntakePosition);
            armLeft.setPower(1.0 / 4);
            armRight.setPower(1.0 / 4);
            armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}
