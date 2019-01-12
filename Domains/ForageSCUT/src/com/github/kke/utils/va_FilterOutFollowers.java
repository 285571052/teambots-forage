package com.github.kke.utils;

import EDU.gatech.cc.is.communication.Message;
import EDU.gatech.cc.is.util.Vec2;

import com.github.hy.utils.*;

public class va_FilterOutFollowers extends va_FilterOutLastPositions {

    public int[] ids;

    /**
     * Instantiate a va_VisualObjects_r node.
     *
     * @param msgs         NodeMsgArray, the raw Messages to be filtered out.
     */
    public va_FilterOutFollowers(NodeMsgArray msgs) {
        super(msgs, PositionsMessageType.FOLLOWER_REPLY);
    }

    @Override
    protected boolean filter(Message message) {
        return message instanceof SourcePositionMessage
                && ((SourcePositionMessage)message).message_type == PositionsMessageType.FOLLOWER_REPLY;
    }

    @Override
    public Vec2[] Value(long timestamp) {
        if ((timestamp > getLasttime()) || (timestamp == -1)) {
            super.Value(timestamp);

            int[] indices = getIndices();
            ids = new int[indices.length];

            Message[] allmsgs = getAllMessages();
            for (int i = 0; i < indices.length; i++) {
                ids[i] = ((SourcePositionMessage)allmsgs[i]).srcid;
            }
        }

        return getLastVal();
    }
}
