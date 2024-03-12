package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Cod_Brat", group = "A")
public class Cod_Brat extends LinearOpMode {
    private DcMotor motorArm1;
    private DcMotor motorArm2;

    private ElapsedTime runtime = new ElapsedTime();

    // Adjust these values based on your robot and gearin
    private final int MAX_POSITION = 0;
    private final int MIN_POSITION = -1000;

    @Override
    public void runOpMode() {
        resetEncoder();
        motorArm1 = hardwareMap.get(DcMotor.class, "motor_arm1");
        motorArm2 = hardwareMap.get(DcMotor.class, "motor_arm2");
        motorArm2.setDirection(DcMotor.Direction.REVERSE);
        motorArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("ticks: ", motorArm1.getCurrentPosition());
            double leftStickX = (double) this.gamepad2.left_stick_y;
            double leftStickY = (double) this.gamepad2.left_stick_x;

            // Gamepad button A for moving the arm up
            if (gamepad1.left_stick_y > 0 && motorArm1.getCurrentPosition() < MAX_POSITION) {
                motorArm1.setPower(1); // Adjust power based on your needs
                motorArm2.setPower(1); // Adjust power based on your needs
            }
            // Gamepad button Y for moving the arm down
            else if (gamepad1.left_stick_y < 0 && motorArm1.getCurrentPosition() > MIN_POSITION) {
                motorArm1.setPower(-1); // Adjust power based on your needs
                motorArm2.setPower(-1); // Adjust power based on your needs
            }
            // Stop the arm if no buttons are pressed
            else {
                motorArm1.setPower(0);
                motorArm2.setPower(0);
            }

            int targetPosition = 0;

            while (gamepad1.a && motorArm1.getCurrentPosition() > targetPosition) {
                motorArm1.setPower(-1);
                motorArm2.setPower(-1);

                motorArm1.setTargetPosition(targetPosition);
                motorArm2.setTargetPosition(targetPosition);

                // Add a short delay to avoid busy-waiting and allow the loop to execute periodically
                sleep(50);
            }
            while (gamepad1.a && motorArm1.getCurrentPosition() < targetPosition) {
                motorArm1.setPower(1); // Adjust power based on your needs
                motorArm2.setPower(1); // Adjust power based on your needs

                motorArm1.setTargetPosition(targetPosition);
                motorArm2.setTargetPosition(targetPosition);

                // Add a short delay to avoid busy-waiting and allow the loop to execute periodically
                sleep(50);
            }

            int targetPosition1 = -20;

            while (gamepad1.b && motorArm1.getCurrentPosition() > targetPosition1) {
                motorArm1.setPower(-1);
                motorArm2.setPower(-1);

                motorArm1.setTargetPosition(targetPosition1);
                motorArm2.setTargetPosition(targetPosition1);

                // Add a short delay to avoid busy-waiting and allow the loop to execute periodically
                sleep(50);
            }
            while (gamepad1.b && motorArm1.getCurrentPosition() < targetPosition1) {
                motorArm1.setPower(1); // Adjust power based on your needs
                motorArm2.setPower(1); // Adjust power based on your needs

                motorArm1.setTargetPosition(targetPosition1);
                motorArm2.setTargetPosition(targetPosition1);

                // Add a short delay to avoid busy-waiting and allow the loop to execute periodically
                sleep(50);
            }

            int targetPosition2 = -400;

            while (gamepad1.x && motorArm1.getCurrentPosition() > targetPosition2) {
                motorArm1.setPower(-1);
                motorArm2.setPower(-1);

                motorArm1.setTargetPosition(targetPosition2);
                motorArm2.setTargetPosition(targetPosition2);

                // Add a short delay to avoid busy-waiting and allow the loop to execute periodically
            }
            while (gamepad1.x && motorArm1.getCurrentPosition() < targetPosition2) {
                motorArm1.setPower(1); // Adjust power based on your needs
                motorArm2.setPower(1); // Adjust power based on your needs

                motorArm1.setTargetPosition(targetPosition2);
                motorArm2.setTargetPosition(targetPosition2);

                // Add a short delay to avoid busy-waiting and allow the loop to execute periodically
            }

            int targetPosition3 = -575;

            while (gamepad1.y && motorArm1.getCurrentPosition() > targetPosition3) {
                motorArm1.setPower(-1);
                motorArm2.setPower(-1);

                motorArm1.setTargetPosition(targetPosition3);
                motorArm2.setTargetPosition(targetPosition3);

                // Add a short delay to avoid busy-waiting and allow the loop to execute periodically
            }
            while (gamepad1.y && motorArm1.getCurrentPosition() < targetPosition3) {
                motorArm1.setPower(1); // Adjust power based on your needs
                motorArm2.setPower(1); // Adjust power based on your needs

                motorArm1.setTargetPosition(targetPosition3);
                motorArm2.setTargetPosition(targetPosition3);

                // Add a short delay to avoid busy-waiting and allow the loop to execute periodically
            }

            telemetry.addData("Arm Position", motorArm1.getCurrentPosition());
            telemetry.update();
        }
    }
    private void resetEncoder() {
        motorArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm1.setTargetPosition(motorArm1.getCurrentPosition());
        motorArm2.setTargetPosition(motorArm2.getCurrentPosition());
        motorArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}