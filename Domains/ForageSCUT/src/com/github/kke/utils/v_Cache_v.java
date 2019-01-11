package com.github.kke.utils;

import EDU.gatech.cc.is.clay.NodeVec2;
import EDU.gatech.cc.is.util.Vec2;

public class v_Cache_v extends NodeVec2 implements CacheValue, CacheNodeValue {

    private NodeVec2 node;
    private Vec2 value = new Vec2();

    public v_Cache_v(NodeVec2 n) {
        super();
        node = n;
    }

    @Override
    public void storeValue(long timestamp) {
        Vec2 v = node.Value(timestamp);
//        System.out.println("("+v.x+","+v.y+") stored");
        value.setx(v.x);
        value.sety(v.y);
    }

    @Override
    public Vec2 retreiveValue() {
//        System.out.println("("+value.x+","+value.y+") retreived.");
		return new Vec2(value.x, value.y);
	}

    @Override
    public Vec2 Value(long timestamp) {
        return retreiveValue();
    }

    @Override
    public void switchNode(NodeVec2 node) {
        this.node = node;
    }

}