package com.github.kke.inertia;

import EDU.gatech.cc.is.util.Vec2;

public interface CachedValue {
    public Vec2 getCached();
    public void setCache(Vec2 v);
}