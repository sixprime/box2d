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

#ifndef B2_DISTANCE_JOINT_H
#define B2_DISTANCE_JOINT_H

#include "b2_api.h"
#include "b2_joint.h"

/// Distance joint definition. This requires defining an anchor point on both
/// bodies and the non-zero distance of the distance joint. The definition uses
/// local anchor points so that the initial configuration can violate the
/// constraint slightly. This helps when saving and loading a game.
struct B2_API b2DistanceJointDef : public b2JointDef
{
	b2DistanceJointDef()
	{
		type = e_distanceJoint;
		localAnchorA.Set(0.0f, 0.0f);
		localAnchorB.Set(0.0f, 0.0f);
		length = 1.0f;
		minLength = 0.0f;
		maxLength = FLT_MAX;
		stiffness = 0.0f;
		damping = 0.0f;
	}

	/// Initialize the bodies, anchors, and rest length using world space anchors.
	/// The minimum and maximum lengths are set to the rest length.
	void Initialize(b2Body* bodyA, b2Body* bodyB,
					const b2Vec2& anchorA, const b2Vec2& anchorB);

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB;

	/// The rest length of this joint. Clamped to a stable minimum value.
	float32 length;

	/// Minimum length. Clamped to a stable minimum value.
	float32 minLength;

	/// Maximum length. Must be greater than or equal to the minimum length.
	float32 maxLength;

	/// The linear stiffness in N/m.
	float32 stiffness;

	/// The linear damping in N*s/m.
	float32 damping;
};

/// A distance joint constrains two points on two bodies to remain at a fixed
/// distance from each other. You can view this as a massless, rigid rod.
class B2_API b2DistanceJoint : public b2Joint
{
public:

	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	/// Get the reaction force given the inverse time step.
	/// Unit is N.
	b2Vec2 GetReactionForce(float32 inv_dt) const override;

	/// Get the reaction torque given the inverse time step.
	/// Unit is N*m. This is always zero for a distance joint.
	float32 GetReactionTorque(float32 inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// Get the rest length
	float32 GetLength() const { return m_length; }

	/// Set the rest length
	/// @returns clamped rest length
	float32 SetLength(float32 length);

	/// Get the minimum length
	float32 GetMinLength() const { return m_minLength; }

	/// Set the minimum length
	/// @returns the clamped minimum length
	float32 SetMinLength(float32 minLength);

	/// Get the maximum length
	float32 GetMaxLength() const { return m_maxLength; }

	/// Set the maximum length
	/// @returns the clamped maximum length
	float32 SetMaxLength(float32 maxLength);

	/// Get the current length
	float32 GetCurrentLength() const;

	/// Set/get the linear stiffness in N/m
	void SetStiffness(float32 stiffness) { m_stiffness = stiffness; }
	float32 GetStiffness() const { return m_stiffness; }

	/// Set/get linear damping in N*s/m
	void SetDamping(float32 damping) { m_damping = damping; }
	float32 GetDamping() const { return m_damping; }

	/// Dump joint to dmLog
	void Dump() override;

	///
	void Draw(b2Draw* draw) const override;

protected:

	friend class b2Joint;
	b2DistanceJoint(const b2DistanceJointDef* data);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	float32 m_stiffness;
	float32 m_damping;
	float32 m_bias;
	float32 m_length;
	float32 m_minLength;
	float32 m_maxLength;

	// Solver shared
	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	float32 m_gamma;
	float32 m_impulse;
	float32 m_lowerImpulse;
	float32 m_upperImpulse;

	// Solver temp
	int32 m_indexA;
	int32 m_indexB;
	b2Vec2 m_u;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	float32 m_currentLength;
	float32 m_invMassA;
	float32 m_invMassB;
	float32 m_invIA;
	float32 m_invIB;
	float32 m_softMass;
	float32 m_mass;
};

#endif
