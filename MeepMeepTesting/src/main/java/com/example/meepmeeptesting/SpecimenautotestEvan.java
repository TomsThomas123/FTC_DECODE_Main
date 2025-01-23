package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SpecimenautotestEvan {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -61, Math.toRadians(90)))

               // .afterTime(0, arm.armUp())
                .strafeTo(new Vector2d(0, -30))
                .waitSeconds(1)

                .setReversed(true)
               // .afterTime(0, arm.armDown())
                .splineToSplineHeading(new Pose2d(new Vector2d(26,-43), Math.toRadians(-90)), 0)


                .splineTo(new Vector2d(45.2, -13), 0)

                .strafeTo(new Vector2d(45,-53))
                //one in observation zone
                .strafeTo(new Vector2d(45,-13))

                .strafeTo(new Vector2d(56,-13))
                .strafeTo(new Vector2d(50,-59))
                .waitSeconds(0.5)
               // .afterTime(0, arm.armUp())

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(0,-35), Math.toRadians(90)), Math.toRadians(90))

                .setReversed(true)
               // .afterTime(0, arm.armDown())
                .splineToSplineHeading(new Pose2d(new Vector2d(38, -59.5), Math.toRadians(-90)), Math.toRadians(0))

                .waitSeconds(0.5)
               // .afterTime(0, arm.armUp())
                .setReversed(true)

                .splineToSplineHeading(new Pose2d(new Vector2d(0,-35), Math.toRadians(90)), Math.toRadians(90))
                /*
                .setReversed(true)
                //.afterTime(0, arm.armDown())
                .splineToSplineHeading(new Pose2d(new Vector2d(38, -59.5), Math.toRadians(-90)), Math.toRadians(0))
                .waitSeconds(0.5)
               // .afterTime(0, arm.armUp())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(0,-35), Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(55,-57), Math.toRadians(-90)), Math.toRadians(0))

                */


                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}