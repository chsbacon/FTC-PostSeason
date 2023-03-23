package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.HMap20342;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="AutoLeftRed", group="Autonomous")

@Config
public class AutoLeftRed extends LinearOpMode {
    public static double d1 = 5.75;
    public static double d2 = 58;
    public static double d3 = 17;
    public static double d5 = 0.1;
    public static double d7 = 2;
    public static double d8 = 12;
    public static double d9 = 19.8;
    public static double d10 = 21;
    public static double d11 = 13;
    public static double d12 = 2;
    public static double d13 = 3;
    public static double d14 = 12;
    public static double t1 = 1;
    public static double finalTurn = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime runtime = new ElapsedTime();
        HMap20342 robot = new HMap20342(hardwareMap);
        Pose2d startPose = new Pose2d(-24-11.5/2, -72+6, Math.toRadians(90)); //5.75

        robot.setPoseEstimate(startPose);
        robot.clawMove(true);

        TrajectorySequence trajSeq = robot.trajectorySequenceBuilder(startPose)
                //drop off preloaded medium
                .addTemporalMarker(t1, ()->robot.armMove("medium"))
                .strafeLeft(d1)
                .forward(d2)
                .back(d3)
                .turn(Math.toRadians(-90))
                .forward(d5)
                .addDisplacementMarker(()->robot.clawMove(true))
                .waitSeconds(0.2)

                //get stack cone
                .addDisplacementMarker(()->robot.armMove("stack"))
                .back(d7)
                .strafeLeft(d8)
                .turn(Math.toRadians(180))
                .forward(d9)
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->robot.clawMove(false))
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()->robot.armMove("medium"))
                .waitSeconds(1)

                //place stack cone
                .back(d10)
                .turn(Math.toRadians(-180))
                .strafeRight(d11)
                .forward(d12)
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->robot.clawMove(true)) //close claw
                .waitSeconds(0.5)

                //start parking
                .addDisplacementMarker(()->robot.armMove("down")) //fast arm better low
                .back(d13)
                .strafeRight(d14)
                .build();
        waitForStart();

        //vision
        HMap20342.SamplePipeline.COLOR color = robot.getColor();
        sleep(50);
        color = robot.getColor();
        telemetry.addData("Color: ", color); telemetry.update();
        runtime.reset();
        while(color == HMap20342.SamplePipeline.COLOR.NONE && runtime.seconds() < 4){
            sleep(50);
            color = robot.getColor();
            telemetry.addData("Color: ", color); telemetry.update();
        }
        if(color == HMap20342.SamplePipeline.COLOR.NONE) color = HMap20342.SamplePipeline.COLOR.ORANGE;

        //auto routine
        robot.clawMove(false);
        robot.followTrajectorySequence(trajSeq);

        if(color == HMap20342.SamplePipeline.COLOR.PINK){ //1
            Trajectory colorTraj = robot.trajectoryBuilder(robot.getPoseEstimate())
                    .back(24)
                    .build();
            robot.followTrajectory(colorTraj);
        }  //2 Orange don't move
        else if(color == HMap20342.SamplePipeline.COLOR.ORANGE) { //3
            Trajectory colorTraj = robot.trajectoryBuilder(robot.getPoseEstimate())
                    .forward(24)
                    .build();
            robot.followTrajectory(colorTraj);
        }
        robot.turn(Math.toRadians(finalTurn));
    }
}