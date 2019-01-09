package com.github.kke.inertia;

import EDU.gatech.cc.is.util.Vec2;

public class CachedVec2 implements CachedValue {

    private Vec2 value = null;

    @Override
    public Vec2 getCached() {
        return value;
    }

    @Override
    public void setCache(Vec2 v) {
		value = new Vec2(v.x, v.y);
	}

}