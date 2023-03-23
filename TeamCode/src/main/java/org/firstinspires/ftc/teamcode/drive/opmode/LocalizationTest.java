package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.HMap20342;
@Config
/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    public static boolean isLeft = true;
    public static boolean isNormal = true;

    @Override
    public void runOpMode() throws InterruptedException {
        HMap20342 drive = new HMap20342(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            double ly = -gamepad1.left_stick_y, lx = -gamepad1.left_stick_x;
            if(!isNormal){
                if(isLeft) ly = 0;
                else lx = 0;
            }
            drive.setWeightedDrivePower(
                    new Pose2d(
                            ly,
                            lx,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            /*telemetry.addData("xpos", drive.twowheel.perpendicularEncoder.getCurrentPosition());
            telemetry.addData("ypos", drive.twowheel.parallelEncoder.getCurrentPosition());
            telemetry.addData("ypos", drive.twowheel.parallelEncoder.getCurrentPosition());*/
            telemetry.addData("error", drive.getLastError());
            telemetry.addData("velocity", drive.getPoseVelocity());
            telemetry.addData("wheelpos", drive.getWheelPositions());
            telemetry.update();
        }
    }
}
