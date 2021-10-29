﻿#ifndef ARIS_DYNAMIC_MODEL_BASE_H_
#define ARIS_DYNAMIC_MODEL_BASE_H_

namespace aris::dynamic
{
	class ARIS_API EndEffector{
	public:
		//auto virtual dim()->Size = 0;
		//auto virtual pos()->double* = 0;
		//auto virtual setPos(double *pos)->void = 0;
		//auto virtual vel()->double* = 0;
		//auto virtual setVel(double *vel)->void = 0;
		//auto virtual acc()->double* = 0;
		//auto virtual setAcc(double *acc)->void = 0;
	};

	class ARIS_API ModelBase{
	public:
		// kinematics & dynamics //
		auto virtual inverseKinematics()->int { return -1; }
		auto virtual forwardKinematics()->int { return -1; }
		auto virtual inverseKinematicsVel()->int { return -1; }
		auto virtual forwardKinematicsVel()->int { return -1; }
		auto virtual inverseDynamics()->int { return -1; }
		auto virtual forwardDynamics()->int { return -1; }

		// inputs //
		auto virtual motionDim()->Size { return 0; }
		auto virtual getMotionPos(double *mp)const ->void { }
		auto virtual setMotionPos(const double *mp)->void { }
		auto virtual getMotionVel(double *mv)const ->void { }
		auto virtual setMotionVel(const double *mv)->void { }
		auto virtual getMotionAcc(double *ma)const ->void { }
		auto virtual setMotionAcc(const double *ma)->void { }
		auto virtual getMotionFce(double *mf)const ->void { }
		auto virtual setMotionFce(const double *mf)->void { }

		// end-effectors //
		auto virtual endEffectorSize()->Size { return 0; }
		auto virtual endEffector(Size i = 0)->EndEffector* { return nullptr; }
	};

	
}

#endif
