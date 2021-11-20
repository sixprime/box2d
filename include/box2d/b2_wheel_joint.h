// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef B2_WHEEL_JOINT_H
#define B2_WHEEL_JOINT_H

#include "b2_api.h"
#include "b2_joint.h"

/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
struct B2_API b2WheelJointDef : public b2JointDef
{
	b2WheelJointDef()
	{
		type = e_wheelJoint;
		localAnchorA.SetZero();
		localAnchorB.SetZero();
		localAxisA.Set(1.0f, 0.0f);
		enableLimit = false;
		lowerTranslation = 0.0f;
		upperTranslation = 0.0f;
		enableMotor = false;
		maxMotorTorque = 0.0f;
		motorSpeed = 0.0f;
		stiffness = 0.0f;
		damping = 0.0f;
	}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.
	void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor, const b2Vec2& axis);

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB;

	/// The local translation axis in bodyA.
	b2Vec2 localAxisA;

	/// Enable/disable the joint limit.
	bool enableLimit;

	/// The lower translation limit, usually in meters.
	float32 lowerTranslation;

	/// The upper translation limit, usually in meters.
	float32 upperTranslation;

	/// Enable/disable the joint motor.
	bool enableMotor;

	/// The maximum motor torque, usually in N-m.
	float32 maxMotorTorque;

	/// The desired motor speed in radians per second.
	float32 motorSpeed;

	/// Suspension stiffness. Typically in units N/m.
	float32 stiffness;

	/// Suspension damping. Typically in units of N*s/m.
	float32 damping;
};

/// A wheel joint. This joint provides two degrees of freedom: translation
/// along an axis fixed in bodyA and rotation in the plane. In other words, it is a point to
/// line constraint with a rotational motor and a linear spring/damper. The spring/damper is
/// initialized upon creation. This joint is designed for vehicle suspensions.
class B2_API b2WheelJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	b2Vec2 GetReactionForce(float32 inv_dt) const override;
	float32 GetReactionTorque(float32 inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// The local joint axis relative to bodyA.
	const b2Vec2& GetLocalAxisA() const { return m_localXAxisA; }

	/// Get the current joint translation, usually in meters.
	float32 GetJointTranslation() const;

	/// Get the current joint linear speed, usually in meters per second.
	float32 GetJointLinearSpeed() const;

	/// Get the current joint angle in radians.
	float32 GetJointAngle() const;

	/// Get the current joint angular speed in radians per second.
	float32 GetJointAngularSpeed() const;

	/// Is the joint limit enabled?
	bool IsLimitEnabled() const;

	/// Enable/disable the joint translation limit.
	void EnableLimit(bool flag);

	/// Get the lower joint translation limit, usually in meters.
	float32 GetLowerLimit() const;

	/// Get the upper joint translation limit, usually in meters.
	float32 GetUpperLimit() const;

	/// Set the joint translation limits, usually in meters.
	void SetLimits(float32 lower, float32 upper);

	/// Is the joint motor enabled?
	bool IsMotorEnabled() const;

	/// Enable/disable the joint motor.
	void EnableMotor(bool flag);

	/// Set the motor speed, usually in radians per second.
	void SetMotorSpeed(float32 speed);

	/// Get the motor speed, usually in radians per second.
	float32 GetMotorSpeed() const;

	/// Set/Get the maximum motor force, usually in N-m.
	void SetMaxMotorTorque(float32 torque);
	float32 GetMaxMotorTorque() const;

	/// Get the current motor torque given the inverse time step, usually in N-m.
	float32 GetMotorTorque(float32 inv_dt) const;

	/// Access spring stiffness
	void SetStiffness(float32 stiffness);
	float32 GetStiffness() const;

	/// Access damping
	void SetDamping(float32 damping);
	float32 GetDamping() const;

	/// Dump to b2Log
	void Dump() override;

	///
	void Draw(b2Draw* draw) const override;

protected:

	friend class b2Joint;
	b2WheelJoint(const b2WheelJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	b2Vec2 m_localXAxisA;
	b2Vec2 m_localYAxisA;

	float32 m_impulse;
	float32 m_motorImpulse;
	float32 m_springImpulse;

	float32 m_lowerImpulse;
	float32 m_upperImpulse;
	float32 m_translation;
	float32 m_lowerTranslation;
	float32 m_upperTranslation;

	float32 m_maxMotorTorque;
	float32 m_motorSpeed;

	bool m_enableLimit;
	bool m_enableMotor;

	float32 m_stiffness;
	float32 m_damping;

	// Solver temp
	int32 m_indexA;
	int32 m_indexB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	float32 m_invMassA;
	float32 m_invMassB;
	float32 m_invIA;
	float32 m_invIB;

	b2Vec2 m_ax, m_ay;
	float32 m_sAx, m_sBx;
	float32 m_sAy, m_sBy;

	float32 m_mass;
	float32 m_motorMass;
	float32 m_axialMass;
	float32 m_springMass;

	float32 m_bias;
	float32 m_gamma;

};

inline float32 b2WheelJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

inline float32 b2WheelJoint::GetMaxMotorTorque() const
{
	return m_maxMotorTorque;
}

#endif
