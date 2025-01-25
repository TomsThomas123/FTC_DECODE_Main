package org.firstinspires.ftc.teamcode.auto;


import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Wrist;
import org.firstinspires.ftc.teamcode.hardware.Claw;

@Config
@Autonomous(name = "RightAutoSpecimen", group = "Autonomous")
public class SpecimenAuto extends LinearOpMode {
    Pose2d startPose;
    MecanumDrive drive;
    Arm arm = new Arm(this);
    Wrist wrist = new Wrist(this);
    Claw claw= new Claw(this);

    @Override
    public void runOpMode() throws InterruptedException {
        startPose = new Pose2d(0, -61, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);
        TrajectoryActionBuilder build = drive.actionBuilder(startPose)
                .afterTime(0.2, arm.armUp())
                .afterTime(0.1, claw.clawClose())
                .strafeTo(new Vector2d(0, -32))
                .afterTime(0.2, wrist.wristScore())
                .waitSeconds(1)



                .afterTime(0.1, arm.armSpec2())



                .setReversed(true)
                .afterTime(0, arm.armDown())
                .splineToSplineHeading(new Pose2d(new Vector2d(26,-43), Math.toRadians(-90)), 0)


                .splineTo(new Vector2d(45, -13), 0)
                /*
                .strafeTo(new Vector2d(45,-53))
                //one in observation zone
                .strafeTo(new Vector2d(45,-13))

                .strafeTo(new Vector2d(55,-13))
                .strafeTo(new Vector2d(55,-59))
                .waitSeconds(0.5)
                .afterTime(0, arm.armUp())

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(0,-35), Math.toRadians(-270)), Math.toRadians(90))
                .setReversed(true)
                .afterTime(0, arm.armDown())
                .splineToSplineHeading(new Pose2d(new Vector2d(38, -59.5), Math.toRadians(270)), Math.toRadians(270))

                .waitSeconds(0.5)
                .afterTime(0, arm.armUp())
                .setReversed(true)

                .splineToSplineHeading(new Pose2d(new Vector2d(0,-35), Math.toRadians(-270)), Math.toRadians(90))
                .setReversed(true)
                .afterTime(0, arm.armDown())
                .splineToSplineHeading(new Pose2d(new Vector2d(38, -59.5), Math.toRadians(270)), Math.toRadians(270))
                .waitSeconds(0.5)
                .afterTime(0, arm.armUp())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(0,-35), Math.toRadians(-270)), Math.toRadians(90))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(55,-57), Math.toRadians(270)), Math.toRadians(270))









                */

                ;
        arm.init();
        wrist.init();
        claw.init();
        int position= 1;
        while (!isStopRequested() && !opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(

            ));


        }

        waitForStart();
        Actions.runBlocking(new SequentialAction(
                build.build()
        ));





    }
}

