
/*
 * forage.java
 */
package com.github.hy.simple;

import EDU.gatech.cc.is.util.Vec2;
import EDU.gatech.cc.is.abstractrobot.*;
import EDU.gatech.cc.is.clay.*;

public class Forage extends ControlSystemMFN150 {
	public final static boolean DEBUG = true;
	private NodeVec2 turret_configuration;
	private NodeVec2 steering_configuration;
	private i_FSA_ba STATE_MACHINE;

	/**
	 * Configure the control system using Clay.
	 */
	public void configure() {
		// ======
		// Set some initial hardware configurations.
		// ======
		abstract_robot.setObstacleMaxRange(3.0); // don't consider
		// things further away
		abstract_robot.setBaseSpeed(0.5 * abstract_robot.MAX_TRANSLATION);

		// ======
		// perceptual schemas
		// ======
		// --- robot's global position
		NodeVec2 PS_GLOBAL_POS = new v_GlobalPosition_r(abstract_robot); // 获取robot全局位置, 通过Value方法

		// --- obstacles
		NodeVec2Array // the sonar readings
		PS_OBS = new va_Obstacles_r(abstract_robot); // 获取感知到障碍物的位置

		// --- targets of visual class 0
		NodeVec2Array PS_TARGETS0_EGO = new va_VisualObjects_r(0, abstract_robot);
		// 获取可视的目标, 判断是否有目标, 用于状态转移判

		NodeVec2Array PS_TARGETS0_GLOBAL = new va_Add_vav(PS_TARGETS0_EGO, PS_GLOBAL_POS);
		// 感知到目标和机器人的相对位置, 相加机器人的标, 获取目标的全局坐标

		// --- make them egocentric
		NodeVec2Array PS_TARGETS0_EGO_FILT = new va_Subtract_vav( // 计算和机器人的相对位置
				PS_TARGETS0_GLOBAL, PS_GLOBAL_POS);

		// --- get the closest one
		NodeVec2 PS_CLOSEST0 = new v_Closest_va(PS_TARGETS0_EGO_FILT); // 获取距离中心最近的目标
		// PS_OBS = new va_remove(PS_OBS, PS_CLOSEST0); //障碍集合去除目标

		// ======
		// Perceptual Features
		// ======
		// is something visible?
		NodeBoolean PF_TARGET0_VISIBLE = new b_NonZero_v(PS_CLOSEST0); // PS_CLOSEST0可能没有目标, 该对象用于判断

		// is it not visible?
		NodeBoolean PF_NOT_TARGET0_VISIBLE = new b_Not_s(PF_TARGET0_VISIBLE); // 继续接PF_TARGET0_VISIBLE...

		// ======
		// motor schemas
		// ======
		// avoid obstacles
		NodeVec2 MS_AVOID_OBSTACLES = new v_Avoid_va(1.5, abstract_robot.RADIUS + 0.1, PS_OBS);
		// 绕障碍物转

		// go to target0
		NodeVec2 MS_MOVE_TO_TARGET0 = new v_LinearAttraction_v(0.4, 0.0, PS_CLOSEST0);
		// 移动到目标需要的向量

		// noise vector
		NodeVec2 MS_NOISE_VECTOR = new v_Noise_(5, seed);
		// 在每个状态的每个动作时, 都增加一定的随机变化量

		// AS_WANDER
		// ======
		v_StaticWeightedSum_va AS_WANDER = new v_StaticWeightedSum_va();

		AS_WANDER.weights[0] = 1.0;
		AS_WANDER.embedded[0] = MS_AVOID_OBSTACLES; // 避开障碍

		AS_WANDER.weights[1] = 1.0;
		AS_WANDER.embedded[1] = MS_NOISE_VECTOR; // 随机移动

		// ======
		// AS_GO_TO_TARGET0
		// ======
		v_StaticWeightedSum_va AS_ACQUIRE0 = new v_StaticWeightedSum_va();

		AS_ACQUIRE0.weights[0] = 1.0;
		AS_ACQUIRE0.embedded[0] = MS_AVOID_OBSTACLES; // 避障

		AS_ACQUIRE0.weights[1] = 2.0;
		AS_ACQUIRE0.embedded[1] = MS_MOVE_TO_TARGET0; // 移向目标

		// ======
		// STATE_MACHINE
		// ======
		STATE_MACHINE = new i_FSA_ba();

		STATE_MACHINE.state = 0;

		// STATE 0 WANDER
		STATE_MACHINE.triggers[0][0] = PF_TARGET0_VISIBLE; // 是否发现目标
		STATE_MACHINE.follow_on[0][0] = 1; // transition to ACQUIRE0

		// STATE 1 ACQUIRE0
		STATE_MACHINE.triggers[1][0] = PF_NOT_TARGET0_VISIBLE;
		STATE_MACHINE.follow_on[1][0] = 0; // transition to WANDER

		// ======
		// STEERING
		// ======
		v_Select_vai STEERING = new v_Select_vai((NodeInt) STATE_MACHINE);

		STEERING.embedded[0] = AS_WANDER;
		STEERING.embedded[1] = AS_ACQUIRE0;

		// ======
		// TURRET
		// ======
		v_Select_vai TURRET = new v_Select_vai((NodeInt) STATE_MACHINE);

		TURRET.embedded[0] = AS_WANDER;
		TURRET.embedded[1] = AS_ACQUIRE0;

		turret_configuration = TURRET;
		steering_configuration = STEERING;
	}

	/**
	 * Called every timestep to allow the control system to run.
	 */
	public int takeStep() {
		Vec2 result;
		long curr_time = abstract_robot.getTime();

		// STEER
		result = steering_configuration.Value(curr_time);
		abstract_robot.setSteerHeading(curr_time, result.t);
		abstract_robot.setSpeed(curr_time, 1.0);

		// TURRET
		result = turret_configuration.Value(curr_time);
		abstract_robot.setTurretHeading(curr_time, result.t);

		// STATE DISPLAY
		int state = STATE_MACHINE.Value(curr_time);
		if (state == 0)
			abstract_robot.setDisplayString("wander");
		else if (state == 1)
			abstract_robot.setDisplayString("acquire");

		return (CSSTAT_OK);
	}
}
