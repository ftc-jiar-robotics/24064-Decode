package org.firstinspires.ftc.teamcode.decode.util;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

public class PathBuilder {

    public static class PathBuilderCallback{
        private final Action action;
        private final double startCondition;
        private final int index;
        private final String name;

        public PathBuilderCallback(Action action, PathChain pathChain, double startCondition, int index, Follower f, String name) {
            this.action = action;
            this.startCondition = startCondition;
            this.index = index;
            this.name = name;
        }
    }
    public static Action buildPath(PathChain pathChain, Follower f, boolean holdEnd, long pathTimeout, PathBuilderCallback... callbacks){

        Action[] callbackActions = new Action[callbacks.length];
        for(int i = 0; i< callbackActions.length;++i) {
            callbackActions[i] = new Actions.CallbackAction(
                    callbacks[i].action,
                    pathChain,
                    callbacks[i].startCondition,
                    callbacks[i].index,
                    f,
                    callbacks[i].name
            );
        }
        Action[] actions = new Action[callbackActions.length + 1];
        System.arraycopy(callbackActions, 0, actions, 0, callbackActions.length);
        actions[callbackActions.length] = pathTimeout < 0 ?
                new FollowPathAction(f, pathChain, holdEnd) :
                new Actions.TimedAction(new FollowPathAction(f, pathChain, holdEnd),pathTimeout,"");

        return new ParallelAction(actions);
    }

    public static Action buildPath(PathChain pathChain, Follower f, boolean holdEnd, PathBuilderCallback... callbacks){
        return buildPath(pathChain,f,holdEnd,-1,callbacks);
    }



}
