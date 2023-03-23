package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

/*
 * Robot goes forward for 0.3 seconds, stop for 0.25,
 * then goes forward again
 */

@TeleOp(name = "TeleOp20342", group = "TeleOp")

public class TeleOp20342 extends LinearOpMode {

    private Map<String, Integer> armLevels = new HashMap<String, Integer>() {{
        put("down", 0);
        put("ground", 429);
        put("low", 5053);
        put("medium", 7868);
        put("high", 11036);
        put("max", 11320);
    }};

    private final ElapsedTime runtime = new ElapsedTime();
    HMap20342TeleOp robot = new HMap20342TeleOp();
    double slowMode = 0.5;

    @Override
    public void runOpMode() { //main method
        robot.init(hardwareMap);

        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {
            double m = Math.hypot(gamepad1.right_stick_x, gamepad1.left_stick_y);
            double tAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
            double turn = -gamepad1.left_stick_x * 0.7;
            robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
            robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            robot.backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            robot.backRightMotor.setDirection(DcMotor.Direction.FORWARD);

            setMecanumDrive(tAngle, m * slowMode, turn);
            telemetry.addLine()
                    .addData("1", robot.armMotor.getCurrentPosition());
            telemetry.update();

            //Slow Mode
            if (gamepad1.right_trigger > 0.75) {
                slowMode = 0.2;
            }
            else if(gamepad1.left_trigger > 0.75){
                slowMode = 1.5;
            }
            else{
                slowMode = 0.7;
            }

            // Claw input
            if (gamepad2.right_bumper) {
                clawMove(true);
                telemetry.addData("Claw closed ", runtime.seconds());
            } else if (gamepad2.left_bumper) {
                clawMove(false);
                telemetry.addData("Claw open ", runtime.seconds());
            }

            // Manual arm functions
            if (gamepad2.dpad_up && robot.armMotor.getCurrentPosition() < armLevels.get("max")) {
                robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.armMotor.setPower(1);
            } else if (gamepad2.dpad_down && robot.armMotor.getCurrentPosition() > armLevels.get("down")) {
                robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.armMotor.setPower(-1);
            } else if (robot.armMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.armMotor.setPower(0);
            }

            // Preset arm functions
          /*  if (gamepad2
                armMove(armLevels.get("ground"));
            else if (gamepad2.x)
                armMove(armLevels.get("low"));
            else if (gamepad2.b)
                armMove(armLevels.get("medium"));
            else if (gamepad2.a)
                armMove(armLevels.get("high"));*/

            telemetry.update();
        }
    }

    private void clawMove(boolean open) {
        int clawPosition = 1;
        if (open) clawPosition = 0;
        robot.clawMotor.setPosition(clawPosition);
        robot.clawMotor2.setPosition(clawPosition);
    }

    private void armMove(int armTicks) {
        robot.armMotor.setTargetPosition(armTicks);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(1);
        // runtime.reset();
        // while(runtime.seconds() < 0.1){}
    }

    public void setMecanumDrive(double tAngle, double m, double turn) {
        // calculate motor power
        double LFpwr = m * Math.cos(tAngle) + turn;
        double LBpwr = m * Math.sin(tAngle) + turn;
        double RFpwr = m * Math.sin(tAngle) - turn;
        double RBpwr = m * Math.cos(tAngle) - turn;

        double turnScale = Math.max(Math.max(Math.abs(LFpwr), Math.abs(LBpwr)),
                Math.max(Math.abs(RFpwr), Math.abs(RBpwr)));
        if (Math.abs(turnScale) < 1.0) turnScale = 1.0;

        telemetry.addData("LF ", LFpwr/turnScale);
        telemetry.addData("RB ", RBpwr/turnScale);

        // set the motors
        robot.frontLeftMotor.setPower(LFpwr / turnScale);
        robot.backRightMotor.setPower(LBpwr / turnScale);
        robot.frontRightMotor.setPower(RFpwr / turnScale);
        robot.backLeftMotor.setPower(RBpwr / turnScale);
    }
}


