package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class aperturesleftauto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-38, -61, Math.toRadians(180)))
                        .waitSeconds(4)
                        .splineToLinearHeading(new Pose2d(new Vector2d(-54.5, -54.5), Math.toRadians(225)), Math.toRadians(225))

                        .waitSeconds(1)


                        .strafeTo(new Vector2d(-54.5, -54.5))

                        //one in hb

                        //pick sample
                        .splineToLinearHeading(new Pose2d(new Vector2d(-48, -46), Math.toRadians(90)), Math.toRadians(90))

                        .waitSeconds(1)


                        .splineToLinearHeading(new Pose2d(new Vector2d(-54.5, -54.5), Math.toRadians(225)), Math.toRadians(225))


                        .waitSeconds(1)

                        .strafeTo(new Vector2d(-54.5, -54.5))
                        //2 in hb

                        //pick sample
                        .splineToLinearHeading(new Pose2d(new Vector2d(-42, -29), Math.toRadians(180)), Math.toRadians(90))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(-44, -29))

                        .waitSeconds(1)

                        .splineToLinearHeading(new Pose2d(new Vector2d(-54.5, -54.5), Math.toRadians(225)), Math.toRadians(225))


                        .waitSeconds(1)
                        .strafeTo(new Vector2d(-54.5, -54.5))
                //3 in hb



                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}