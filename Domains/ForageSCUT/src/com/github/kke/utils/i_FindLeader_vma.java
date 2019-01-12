package com.github.kke.utils;

import EDU.gatech.cc.is.clay.NodeInt;
import EDU.gatech.cc.is.clay.NodeVec2;
import EDU.gatech.cc.is.communication.Message;
import EDU.gatech.cc.is.util.Vec2;
import com.github.hy.utils.*;

public class i_FindLeader_vma extends NodeInt {
    private NodeMsgArray nodeMsgArray;
    private NodeVec2 choice;
    private int last_value = NO_MATCH_LEADER;
    private long lasttime = 0;

    private final static int NO_MATCH_LEADER = -1;
    private final static double EPSILON = 0.00000000001;

    public i_FindLeader_vma(NodeVec2 choice, NodeMsgArray msgArray) {
        this.nodeMsgArray = msgArray;
        this.choice = choice;
    }

    /**
     *
     * @param timestamp long indicates time of the request
     * @return leader's id
     */
    @Override
    public int Value(long timestamp) {
        if ((timestamp > lasttime) || (timestamp == -1)) {
            if (timestamp > 0)
                lasttime = timestamp;

            last_value = NO_MATCH_LEADER; // 一般情况下不应出现

            Vec2 v = choice.Value(timestamp);

            Message[] msgs = nodeMsgArray.Value(timestamp);
            for (int i = msgs.length-1; i >= 0; i--) {
                if (msgs[i] instanceof SourcePositionMessage) {
                    SourcePositionMessage spm = (SourcePositionMessage)msgs[i];
                    if (spm.message_type == PositionsMessageType.LEADER && containedInMsg(v, spm)) {
                        last_value = spm.srcid;
                        break;
                    }
                }
            }
        }

        return last_value;
    }

    private static boolean vectorEquals(Vec2 a, Vec2 b) {
        return Math.abs(a.x - b.x) < EPSILON && Math.abs(a.y - b.y) < EPSILON;
    }

    private static boolean containedInMsg(Vec2 b, PositionsMessage pm) {
        for (Vec2 a : pm.val) {
            if (vectorEquals(a, b)) {
                return true;
            }
        }
        return false;
    }
}
