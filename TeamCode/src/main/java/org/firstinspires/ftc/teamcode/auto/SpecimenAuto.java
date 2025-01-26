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
import org.firstinspires.ftc.teamcode.hardware.Lift;

@Config
@Autonomous(name = "RightAutoSpecimen", group = "Autonomous")
public class SpecimenAuto extends LinearOpMode {
    Pose2d startPose;
    MecanumDrive drive;
    Arm arm = new Arm(this);
    Wrist wrist = new Wrist(this);
    Claw claw= new Claw(this);
    Lift lift = new Lift(this);

    @Override
    public void runOpMode() throws InterruptedException {
        startPose = new Pose2d(0, -61, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);
        TrajectoryActionBuilder build = drive.actionBuilder(startPose)
                .afterTime(0.2, arm.armUp())
                .afterTime(0.1, claw.clawClose())
                .strafeTo(new Vector2d(0, -31.4))
                .afterTime(0.1, wrist.wristScore())
                .strafeTo(new Vector2d(0, -31))
                .afterTime(0 , wrist.wristScore2())
                .afterTime(0, wrist.wristDanger())
                .afterTime(0 , arm.armSpec2())
                .afterTime(0, wrist.wristDanger())
                .waitSeconds(0.8)
                .afterTime(0 , wrist.wristDanger())
               /* .afterTime(0, arm.armSpec2())
                .afterTime(0, wrist.wristDanger())
                .afterTime(0, wrist.wristScore())
                .afterTime(0 , wrist.wristScore2())
                .afterTime(0, wrist.wristDanger())
                .waitSeconds(2)

                */
                .afterTime(1, claw.clawOpen())

                //first specimen done
                .waitSeconds(0.8)
                .afterTime(0.1, wrist.wristDown())
                .setReversed(true)
                .afterTime(0 , arm.armStop())
                .splineToSplineHeading(new Pose2d(new Vector2d(26,-43), Math.toRadians(-90)), 0)


                .splineTo(new Vector2d(45, -13), 0)

                .strafeTo(new Vector2d(45,-53))
                //one in observation zone
                .strafeTo(new Vector2d(45,-13))

                .strafeTo(new Vector2d(56.5,-13))
                .afterTime(0, wrist.wristGrab())
                .strafeTo(new Vector2d(56.5,-55))
                .waitSeconds(1)
                .afterTime(0.1, claw.clawClose())
                .waitSeconds(0.9)
                .afterTime(1, arm.armUp())
                .afterTime(0, wrist.wristDown())

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(1,-31.3), Math.toRadians(-270)), Math.toRadians(90))
                .afterTime(0.1, wrist.wristScore())
                .strafeTo(new Vector2d(0, -31))
                .afterTime(0 , wrist.wristScore2())
                .afterTime(0, wrist.wristDanger())
                .afterTime(0 , arm.armSpec2())
                .afterTime(0, wrist.wristDanger())
                .waitSeconds(0.8)
                .afterTime(0, claw.clawOpen())
                .setReversed(true)
                .afterTime(0, arm.armDown())
                .afterTime(0 , arm.armStop())
                .afterTime(0, wrist.wristGrab())
                .splineToSplineHeading(new Pose2d(new Vector2d(38, -53), Math.toRadians(270)), Math.toRadians(270))
                .afterTime(0, wrist.wristGrab())
                .waitSeconds(1)
                .afterTime(0.1, claw.clawClose())
                .waitSeconds(0.9)
                .afterTime(1, arm.armUp())
                .afterTime(0, wrist.wristDown())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(2,-31.3), Math.toRadians(-270)), Math.toRadians(90))
                // commented for test .afterTime(0.1, wrist.wristScore())
                .strafeTo(new Vector2d(0, -31.1))
                .afterTime(0.1, wrist.wristScore()) //this is added as test delete this line and uncomment other one for 2 spec auto
                .afterTime(0 , wrist.wristScore2())
                .afterTime(0, wrist.wristDanger())
                .afterTime(0 , arm.armSpec2())
                .afterTime(0, wrist.wristDanger())
                .waitSeconds(0.8)
                .afterTime(0, claw.clawOpen())
                .setReversed(true)
                .afterTime(0, arm.armDown())
                .afterTime(0 , arm.armStop())
                .splineToSplineHeading(new Pose2d(new Vector2d(38, -55), Math.toRadians(270)), Math.toRadians(270))
                .afterTime(0, wrist.wristGrab())
                .waitSeconds(1)
                .afterTime(0.1, claw.clawClose())
                .waitSeconds(1)

                .afterTime(1, arm.armUp())
                .afterTime(0, wrist.wristDown())

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(1,-31.3), Math.toRadians(-270)), Math.toRadians(90))
                .afterTime(0.1, wrist.wristScore())
                .strafeTo(new Vector2d(0, -31.1))
                .afterTime(0 , wrist.wristScore2())
                .afterTime(0, wrist.wristDanger())
                .afterTime(0 , arm.armSpec2())
                .afterTime(0, wrist.wristDanger())
                .waitSeconds(1)
                .afterTime(0, claw.clawOpen())
                .setReversed(true)
                .afterTime(0, arm.armDown())
                .afterTime(0 , arm.armStop())


                .splineToSplineHeading(new Pose2d(new Vector2d(55,-55), Math.toRadians(270)), Math.toRadians(270))











                ;
        arm.init();
        wrist.init();
        claw.init();
        lift.init();
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

