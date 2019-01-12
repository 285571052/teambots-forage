
/*
 * va_FilterOutLastPositions.java
 */
package com.github.hy.utils;

import EDU.gatech.cc.is.clay.*;
import EDU.gatech.cc.is.util.Vec2;
import EDU.gatech.cc.is.communication.*;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public class va_FilterOutLastPositions extends NodeVec2Array {
    /**
     * Turn debug printing on or off.
     */
    public static final boolean DEBUG = false;
    NodeMsgArray msgs;

    Vec2 last_val[] = new Vec2[0];
    long lasttime = 0;
    PositionsMessageType message_type;

    private int[] indices;
    private Message[] allMsgs;

    /**
     * Instantiate a va_VisualObjects_r node.
     *
     * @param msgs NodeMsgArray, the raw Messages to be filtered out.
     */
    public va_FilterOutLastPositions(NodeMsgArray msgs, PositionsMessageType message_type) {
        if (DEBUG)
            System.out.println("va_FilterOutLastPositions: instantiated");
        this.msgs = msgs;
        this.message_type = message_type;
    }

    private void filterOutIndices() {
        int[] temp_indices = new int[allMsgs.length];
        int len = 0;
        for (int i = allMsgs.length-1; i >= 0 ; i--)
            if (filter(allMsgs[i]))
                temp_indices[len++] = i;

        indices = new int[len];
        for (int i = 0; i < len; i++)
            indices[i] = temp_indices[i];
    }

    protected boolean filter(Message message) {
        return message instanceof PositionsMessage && ((PositionsMessage)message).message_type == message_type;
    }

    /**
     * Return an array of Positions from the last PositionsMessage.
     *
     * @param timestamp long, only get new information if timestamp > than last call
     *                  or timestamp == -1.
     * @return the messages recieved
     */
    public Vec2[] Value(long timestamp) {
        if ((timestamp > lasttime) || (timestamp == -1)) {
            /*--- reset the timestamp ---*/
            if (timestamp > 0) {
                lasttime = timestamp;
            }

            /*--- count ---*/
            allMsgs = msgs.Value(timestamp);

            filterOutIndices();
            Set<Vec2> positions = new HashSet<>();

            for (int index: indices) {
                PositionsMessage pm = (PositionsMessage)allMsgs[index];
                Collections.addAll(positions, pm.val);
            }

            last_val = new Vec2[positions.size()];
            last_val = positions.toArray(last_val);
        }

        Vec2[] retval = new Vec2[last_val.length];
        for (int i = 0; i < last_val.length; i++)
            retval[i] = new Vec2(last_val[i].x, last_val[i].y);

        return (retval);
    }

    protected int[] getIndices() {
        return indices;
    }

    protected long getLasttime() {
        return lasttime;
    }

    protected Message[] getAllMessages() {
        return allMsgs;
    }

    protected Vec2[] getLastVal() {
        return last_val;
    }
}
