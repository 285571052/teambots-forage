package com.github.kke.utils;

import EDU.gatech.cc.is.util.Vec2;
import com.github.hy.utils.PositionsMessage;
import com.github.hy.utils.PositionsMessageType;

public class SourcePositionMessage extends PositionsMessage {

    public final static int NO_ID = -1;
    public int srcid = NO_ID;

    public SourcePositionMessage(Vec2[] p, PositionsMessageType message_type, int id) {
        super(p, message_type);
        srcid = id;
    }

    public SourcePositionMessage(Vec2 v, PositionsMessageType message_type, int id) {
        this(vectorToArray(v), message_type, id);
    }

    private static Vec2[] vectorToArray(Vec2 v) {
        Vec2[] p = new Vec2[1];
        p[0] = new Vec2(v.x, v.y);

        return p;
    }
}
