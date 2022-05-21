#include "cSceneKickBallAMP.h"
#include "sim/SimSphere.h"
#define M_PI 3.14159265358979323846
const double cSceneKickBallAMP::gComBallDistThreshold = 2;

double cSceneKickBallAMP::CalcReward(int agent_id) const
{
	double reward = 0;
	if (mMode == eModeTest)
	{
		reward = CalcRewardTest(agent_id);
	}
	else
	{
		reward = CalcRewardTrain(agent_id);
	}
	return reward;
}

double cSceneKickBallAMP::CalcRewardTrain(int agent_id) const
{
	double tar_dribble_w = 0.5;
	double tar_near_w = 0.2;
	double tar_goal_w = 0.3;
	double goal_reward = 0.1;

	double reward = 0;

	const auto& ctrl = GetController(agent_id);
	const cSimCharacter* character = GetAgentChar(agent_id);
	bool fallen = HasFallen(*character);
	tVector root_pos = character->GetRootPos();
	tVector ball_pos = GetBallPos();
	tVector tar_pos = GetTargetPos();//mKickTargetPos;//GetTargetPos();
	double near_dist = GetTarNearDist();

	tVector tar_root_delta = tar_pos - root_pos;
	tVector ball_root_delta = ball_pos - root_pos;

	double tar_root_dist_sq = tar_root_delta[0] * tar_root_delta[0] + tar_root_delta[2] * tar_root_delta[2];
	double ball_root_dist_sq = ball_root_delta[0] * ball_root_delta[0] + ball_root_delta[2] * ball_root_delta[2];

	
	if (!fallen)
	{
		if (mMarkHit)
		{
			double tar_goal_r = CalcRewardTargetGoal(root_pos);

			reward = tar_dribble_w + tar_near_w + tar_goal_w * tar_goal_r;

			if(CheckTargetSucc())
				reward += 0.1;
		}
		else if (CheckCanKick()) // near
		{
			
			double tar_near_r = CalcRewardTargetdNear(agent_id);
			reward = tar_dribble_w + tar_near_w * tar_near_r;

		}
		else
		{
			double tar_dribble_r = CalcRewardDribble(agent_id);
			reward = tar_dribble_w * tar_dribble_r;
		}
	}

	return reward;
}

double cSceneKickBallAMP::CalcRewardTest(int agent_id) const
{
	double reward = 0.0;
	bool episode_end = IsEpisodeEnd();
	if (episode_end)
	{
		eTerminate term = CheckTerminate(agent_id);
		if (term == eTerminateSucc)
		{
			reward = mTimer.GetMaxTime() - GetTime();
		}
	}
	return reward;
}


double cSceneKickBallAMP::CalcRewardTargetGoal(const tVector& rootPos) const
{
	double reward = 0.0;

	tVector ball_pos = GetBallPos();
	tVector goal_pos = GetTargetPos();

	tVector tar_player_delta = goal_pos - rootPos;

	tVector tar_body_delta = goal_pos - ball_pos;

	double dist_sq = (tar_body_delta).squaredNorm();
	double dist_sq_p = (GetCanKickDist() - tar_player_delta.squaredNorm());
	if(dist_sq_p < 0) dist_sq_p = 0;

	reward = 0.5 * std::exp(-4.0 * dist_sq) + 0.5 * std::exp(-4.0 * dist_sq_p);

	return reward;
}

double cSceneKickBallAMP::CalcRewardDribble(int agent_id) const
{
	const double desired_com_ball_vel = GetTargetSpeed();
	const double desired_ball_target_vel = GetTargetSpeed();

	double com_ball_vel_w = 0.1;
	double com_ball_pos_w = 0.1;
	double target_vel_w = 0.3;
	double target_pos_w = 0.5;

	const double total_w = com_ball_vel_w + com_ball_pos_w + target_vel_w + target_pos_w;
	com_ball_vel_w /= total_w;
	com_ball_pos_w /= total_w;
	target_vel_w /= total_w;
	target_pos_w /= total_w;

	const double com_ball_vel_scale = 1.5;
	const double com_ball_pos_scale = 0.5;
	const double target_vel_scale = 1;
	const double target_pos_scale = 0.5;

	double reward = 0;
	const auto& ctrl = GetController(agent_id);
	const cSimCharacter* character = GetAgentChar(agent_id);

	const cDeepMimicCharController* char_ctrl = dynamic_cast<const cDeepMimicCharController*>(ctrl.get());
	double prev_action_time = char_ctrl->GetPrevActionTime();
	const tVector& prev_action_com = char_ctrl->GetPrevActionCOM();
	const tVector& prev_ball_pos = mAgentPrevTarObjPos[agent_id];

	double time_elapsed = char_ctrl->GetTime() - prev_action_time;

	tVector curr_com = character->CalcCOM();
	tVector ball_pos = GetBallPos();

	tVector com_delta = curr_com - prev_action_com;
	tVector com_ball_delta = ball_pos - prev_action_com;
	com_delta[1] = 0;
	com_ball_delta[1] = 0;
	tVector com_ball_dir = com_ball_delta.normalized();

	double com_ball_dist = com_ball_delta.squaredNorm();
	double com_ball_vel = com_ball_dir.dot(com_delta);
	com_ball_vel /= time_elapsed;
	double com_ball_vel_err = std::min(0.0, com_ball_vel - desired_com_ball_vel);
	com_ball_vel_err *= com_ball_vel_err;

	double com_ball_pos_err = com_ball_dist;

	tVector ball_delta = ball_pos - prev_ball_pos;
	tVector ball_target_delta = GetTargetPos() - prev_ball_pos;
	ball_delta[1] = 0;
	ball_target_delta[1] = 0;
	tVector ball_target_dir = ball_target_delta.normalized();

	double ball_target_vel = ball_target_dir.dot(ball_delta);
	ball_target_vel /= time_elapsed;
	double target_vel_err = std::min(0.0, ball_target_vel - desired_ball_target_vel);
	target_vel_err *= target_vel_err;

		double curr_ball_target_dist = (GetTargetPos() - ball_pos).squaredNorm();
		double target_pos_err = std::sqrt(curr_ball_target_dist);

		double com_ball_vel_reward = std::exp(-com_ball_vel_scale * com_ball_vel_err);
		double com_ball_pos_reward = std::exp(-com_ball_pos_scale * com_ball_pos_err);
		double target_vel_reward = std::exp(-target_vel_scale * target_vel_err);
		double target_pos_reward = std::exp(-target_pos_scale * target_pos_err);

		bool target_success = (curr_ball_target_dist < mTargetSuccDist* mTargetSuccDist)
			&& (com_ball_dist < gComBallDistThreshold* gComBallDistThreshold);
		if (target_success)
		{
			com_ball_vel_reward = 1;
			com_ball_pos_reward = 1;
			target_vel_reward = 1;
			target_pos_reward = 1;
		}

		reward = com_ball_vel_w * com_ball_vel_reward + com_ball_pos_w * com_ball_pos_reward
			+ target_vel_w * target_vel_reward + target_pos_w * target_pos_reward;
	

	return reward;
}


double cSceneKickBallAMP::CalcRewardTargetdNear(int agent_id) const
{
	double reward = 0.0;

	const cSimCharacter* sim_char = GetAgentChar(agent_id);
	tVector root_pos = sim_char->GetRootPos();
	tVector target_pos = mKickTargetPos;//GetTargetPos();
	tVector tar_root_delta = target_pos - root_pos;
	tar_root_delta[1] = 0.0;
	double norm = tar_root_delta.norm();

	tVector tar_root_dir = tVector::Zero();
	if (norm > 1e-5)
	{
		tar_root_dir = tar_root_delta / norm;
	}

	for (int i = 0; i < GetNumStrikeBodies(); ++i)
	{
		int b = mStrikeBodies[i];
		const auto& body_part = sim_char->GetBodyPart(b);
		const tVector pos = body_part->GetPos();
		const tVector vel = body_part->GetLinearVelocity();

		tVector tar_body_delta = target_pos - pos;
		double dist_sq = (tar_body_delta).squaredNorm();
		double curr_dist_reward = std::exp(-mTarRewardScale * dist_sq);

		double tar_body_speed = tar_root_dir.dot(vel);
		double cur_vel_reward = tar_body_speed / mHitTarSpeed;
		cur_vel_reward = cMathUtil::Clamp(cur_vel_reward, 0.0, 1.0);
		cur_vel_reward *= cur_vel_reward;

		double curr_reward = 0.2 * curr_dist_reward + 0.8 * cur_vel_reward;
		reward = std::max(reward, curr_reward);
	}

	return reward;
}

double cSceneKickBallAMP::CalcRewardTargetFar(int agent_id) const
{
	const double pos_reward_w = 0.7;
	const double vel_reward_w = 0.3;
	const bool enable_min_tar_vel = mEnableMinTarVel;
	const double near_dist = GetTarNearDist();

	const double tar_speed = GetTargetSpeed();
	const double pos_err_scale = mPosRewardScale;
	const double vel_err_scale = 4 / (tar_speed * tar_speed);

	double reward = 0;

	const auto& ctrl = GetController(agent_id);
	const cSimCharacter* character = GetAgentChar(agent_id);
	bool fallen = HasFallen(*character);
	if (!fallen)
	{
		const cDeepMimicCharController* trl_ctrl = dynamic_cast<const cDeepMimicCharController*>(ctrl.get());
		double prev_action_time = trl_ctrl->GetPrevActionTime();
		const tVector& prev_action_com = trl_ctrl->GetPrevActionCOM();

		const tVector& tar_pos = mKickTargetPos;//GetTargetPos();
		tVector root_pos = character->GetRootPos();
		tVector root_tar_delta = tar_pos - root_pos;
		root_tar_delta[1] = 0.0;

		double root_tar_dist = root_tar_delta.norm();
		double root_dist_err = root_tar_dist - near_dist;
		root_dist_err = std::max(root_dist_err, 0.0);
		double pos_reward = std::exp(-pos_err_scale * root_dist_err * root_dist_err);

		double vel_reward = 0;

		if (root_tar_dist < near_dist)
		{
			vel_reward = 1.0;
		}
		else
		{
			double step_dur = trl_ctrl->GetTime() - prev_action_time;

			tVector com = character->CalcCOM();
			tVector com_tar_delta = tar_pos - com;
			com_tar_delta[1] = 0.0;
			double com_tar_dist = com_tar_delta.norm();

			tVector com_tar_dir = tVector(0, 0, 0, 0);
			if (com_tar_dist > 0.0001)
			{
				com_tar_dir = com_tar_delta / com_tar_dist;
			}

			double avg_vel = com_tar_dir.dot(com - prev_action_com) / step_dur;
			double vel_err = tar_speed - avg_vel;

			if (avg_vel < 0)
			{
				vel_reward = 0.0;
			}
			else
			{
				if (enable_min_tar_vel)
				{
					vel_err = std::max(vel_err, 0.0);
				}
				vel_reward = std::exp(-vel_err_scale * vel_err * vel_err);
			}
		}

		reward = pos_reward_w * pos_reward + vel_reward_w * vel_reward;
	}

	return reward;
}

cSceneKickBallAMP::cSceneKickBallAMP()
{
	mTargetTimerParams.mTimeMin = 50;
	mTargetTimerParams.mTimeMax = 100;
	mTarObjTimerParams.mTimeMin = 100;
	mTarObjTimerParams.mTimeMax = 200;
	mMinTarObjDist = 0.5;
	mMaxTarObjDist = 10;

	mBallRadius = 0.2;
	mTarNearDist = 1.4;
	mHitTarSpeed = 1.5;
}

cSceneKickBallAMP::~cSceneKickBallAMP()
{
}

void cSceneKickBallAMP::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cSceneTargetAMP::ParseArgs(parser);

	parser->ParseDouble("rand_tar_obj_time_min", mTarObjTimerParams.mTimeMin);
	parser->ParseDouble("rand_tar_obj_time_max", mTarObjTimerParams.mTimeMax);
	parser->ParseDouble("min_tar_obj_dist", mMinTarObjDist);
	parser->ParseDouble("max_tar_obj_dist", mMaxTarObjDist);

	parser->ParseInts("strike_bodies", mStrikeBodies);
	parser->ParseDouble("ball_radius", mBallRadius);
	parser->ParseDouble("hit_tar_speed", mHitTarSpeed);
	parser->ParseDouble("can_kick_dist", mCanKickDist);
	parser->ParseDouble("goal_limit_time", mGoalLimitTime);

}

void cSceneKickBallAMP::Init()
{
	cSceneTargetAMP::Init();

	InitTarObjs();
	InitAgentTarObjRecord();

	mTargetTimer.Reset();
	ResetTarget();
}

void cSceneKickBallAMP::Reset()
{
	mTarObjTimer.Reset();
	ResetTarObjs();
	ResetAgentTarObjRecord();
	mIsFail = false;
	cSceneTargetAMP::Reset();
}

void cSceneKickBallAMP::SetBallPos(const tVector& pos)
{
	SetTarObjPos(GetTarObjID(), pos);
	ResetAgentTarObjRecord();
}

tVector cSceneKickBallAMP::GetBallPos() const
{
	tVector pos = tVector::Zero();
	int tar_obj_id = GetTarObjID();
	if (tar_obj_id != gInvalidIdx)
	{
		const auto& ball = GetObj(tar_obj_id);
		pos = ball->GetPos();
	}
	return pos;
}

int cSceneKickBallAMP::GetTarObjID() const
{
	return mTarObjID;
}

void cSceneKickBallAMP::RecordState(int agent_id, Eigen::VectorXd& out_state) const
{
	Eigen::VectorXd ctrl_s;
	Eigen::VectorXd task_s;
	RecordCtrlState(agent_id, ctrl_s);
	RecordTaskState(agent_id, task_s);

	int s_size = GetStateSize(agent_id);
	int ctrl_s_size = GetCtrlStateSize(agent_id);
	int task_s_size = GetTaskStateSize(agent_id);
	assert(s_size == ctrl_s_size + task_s_size);
	assert(ctrl_s_size == ctrl_s.size());
	assert(task_s_size == task_s.size());

	out_state.resize(s_size);
	out_state.segment(0, ctrl_s_size) = ctrl_s;
	out_state.segment(ctrl_s_size, task_s_size) = task_s;
}

int cSceneKickBallAMP::GetStateSize(int agent_id) const
{
	int ctrl_s_size = GetCtrlStateSize(agent_id);
	int task_s_size = GetTaskStateSize(agent_id);
	int s_size = ctrl_s_size + task_s_size;
	return s_size;
}

void cSceneKickBallAMP::BuildStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	Eigen::VectorXd offset_ctrl_s;
	Eigen::VectorXd scale_ctrl_s;
	Eigen::VectorXd offset_task_s;
	Eigen::VectorXd scale_task_s;
	BuildCtrlStateOffsetScale(agent_id, offset_ctrl_s, scale_ctrl_s);
	BuildTaskStateOffsetScale(agent_id, offset_task_s, scale_task_s);

	int s_size = GetStateSize(agent_id);
	int ctrl_s_size = GetCtrlStateSize(agent_id);
	int task_s_size = GetTaskStateSize(agent_id);
	assert(s_size == ctrl_s_size + task_s_size);
	assert(ctrl_s_size == offset_ctrl_s.size());
	assert(task_s_size == offset_task_s.size());
	assert(ctrl_s_size == scale_ctrl_s.size());
	assert(task_s_size == scale_task_s.size());

	out_offset.resize(s_size);
	out_offset.segment(0, ctrl_s_size) = offset_ctrl_s;
	out_offset.segment(ctrl_s_size, task_s_size) = offset_task_s;

	out_scale.resize(s_size);
	out_scale.segment(0, ctrl_s_size) = scale_ctrl_s;
	out_scale.segment(ctrl_s_size, task_s_size) = scale_task_s;
}

void cSceneKickBallAMP::BuildStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	Eigen::VectorXi ctrl_groups;
	Eigen::VectorXi task_groups;
	BuildCtrlStateNormGroups(agent_id, ctrl_groups);
	BuildTaskStateNormGroups(agent_id, task_groups);

	int s_size = GetStateSize(agent_id);
	int ctrl_s_size = GetCtrlStateSize(agent_id);
	int task_s_size = GetTaskStateSize(agent_id);
	assert(s_size == ctrl_s_size + task_s_size);
	assert(ctrl_s_size == ctrl_groups.size());
	assert(task_s_size == task_groups.size());

	out_groups.resize(s_size);
	out_groups.segment(0, ctrl_s_size) = ctrl_groups;
	out_groups.segment(ctrl_s_size, task_s_size) = task_groups;
}

void cSceneKickBallAMP::RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const
{
	int g_size = GetGoalSize(agent_id);
	out_goal = std::numeric_limits<double>::quiet_NaN() * Eigen::VectorXd::Ones(g_size);

	const auto& sim_char = GetAgentChar(agent_id);
	tMatrix origin_trans = sim_char->BuildOriginTrans();
	tVector tar_pos = GetTargetPos();
	tVector ball_pos = GetBallPos();

	tVector rel_tar_pos = tar_pos - ball_pos;
	rel_tar_pos[1] = 0;

	double tar_dist = rel_tar_pos.norm();
	if (tar_dist > 0.0001)
	{
		rel_tar_pos = origin_trans * rel_tar_pos;
		rel_tar_pos /= tar_dist;
	}
	else
	{
		rel_tar_pos = tVector(1, 0, 0, 0);
	}

	out_goal[0] = rel_tar_pos[0];
	out_goal[1] = rel_tar_pos[2];
	out_goal[2] = tar_dist;
}

std::string cSceneKickBallAMP::GetName() const
{
	return "kick_ball_amp";
}


void cSceneKickBallAMP::UpdateObjs(double timestep)
{
	cSceneTargetAMP::UpdateObjs(timestep);

	UpdateTarObjs(timestep);
	if (mTarObjTimer.IsEnd())
	{
		mTarObjTimer.Reset();
	}
}

void cSceneKickBallAMP::InitAgentTarObjRecord()
{
	int num_agents = GetNumAgents();
	mAgentPrevTarObjPos.resize(num_agents);
	ResetAgentTarObjRecord();
}

void cSceneKickBallAMP::ResetAgentTarObjRecord()
{
	for (size_t i = 0; i < mAgentPrevTarObjPos.size(); ++i)
	{
		UpdateAgentTarObjRecord(i);
	}
}

void cSceneKickBallAMP::UpdateAgentTarObjRecord(int agent_id)
{
	tVector ball_pos = GetBallPos();
	mAgentPrevTarObjPos[agent_id] = ball_pos;
}

double cSceneKickBallAMP::GetBallRadius() const
{
	return mBallRadius;
}

void cSceneKickBallAMP::NewActionUpdate(int agent_id)
{
	cSceneTargetAMP::NewActionUpdate(agent_id);
	UpdateAgentTarObjRecord(agent_id);
}

bool cSceneKickBallAMP::HasFallen(const cSimCharacter& sim_char) const
{
	bool fallen = cSceneTargetAMP::HasFallen(sim_char);
	fallen |= CheckTarObjDistFail();
	fallen |= CheckCharObjDistFail(sim_char);
	return fallen;
}

bool cSceneKickBallAMP::CheckTarObjDistFail() const
{
	const double tar_max_dist = GetMaxTargetDist();
	double dist_threshold = 2 * tar_max_dist;

	const tVector& tar_pos = GetTargetPos();
	const tVector& ball_pos = GetBallPos();
	tVector ball_tar_delta = tar_pos - ball_pos;
	ball_tar_delta[1] = 0;
	double ball_tar_dist = ball_tar_delta.squaredNorm();

	bool fail = (ball_tar_dist > dist_threshold * dist_threshold);
	return fail;
}

bool cSceneKickBallAMP::CheckCharObjDistFail(const cSimCharacter& sim_char) const
{
	const double obj_max_dist = GetTarObjMaxDist();
	double dist_threshold = 2 * obj_max_dist;

	const tVector& char_pos = sim_char.GetRootPos();
	const tVector& ball_pos = GetBallPos();
	tVector char_ball_delta = ball_pos - char_pos;
	char_ball_delta[1] = 0;
	double char_ball_dist = char_ball_delta.squaredNorm();

	bool fail = (char_ball_dist > dist_threshold * dist_threshold);
	return fail;
}

void cSceneKickBallAMP::ClearObjs()
{
	cSceneTargetAMP::ClearObjs();
	mTarObjID = gInvalidIdx;
}

void cSceneKickBallAMP::InitTarObjs()
{
	mTarObjTimer.Init(mTarObjTimerParams);
	BuildTarObjs();
	ResetTarObjs();
}

void cSceneKickBallAMP::BuildTarObjs()
{
	const double r = GetBallRadius();
	const double mass = 0.43;
	const double linear_damping = 0.4;
	const double angular_damping = 0.4;
	const double friction = 0.4;

	cSimSphere::tParams params;
	params.mRadius = r;
	params.mPos = tVector(1, 1, 1, 0);
	params.mVel = tVector::Zero();
	params.mFriction = friction;
	params.mMass = mass;

	std::shared_ptr<cSimSphere> ball = std::shared_ptr<cSimSphere>(new cSimSphere());
	ball->Init(mWorld, params);
	ball->UpdateContact(cWorld::eContactFlagObject, cContactManager::gFlagNone);
	ball->SetDamping(linear_damping, angular_damping);

	tObjEntry obj_entry;
	obj_entry.mObj = ball;
	obj_entry.mColor = tVector(0.9, 0.9, 0.9, 1);
	obj_entry.mPersist = true;
	mTarObjID = AddObj(obj_entry);
}

void cSceneKickBallAMP::ResetTarObjs()
{
	const double min_dist = GetTarObjMinDist();
	const double max_dist = GetTarObjMaxDist();
	assert(min_dist <= max_dist);

	const auto& sim_char = GetCharacter();

	tVector rand_pos = tVector::Zero();
	tVector root_pos = sim_char->GetRootPos();
	double r = mRand.RandDouble(min_dist, max_dist);
	double theta = mRand.RandDouble(-M_PI, M_PI);
	rand_pos[0] = root_pos[0] + r * std::cos(theta);
	rand_pos[2] = root_pos[2] + r * std::sin(theta);

	SetTarObjPos(GetTarObjID(), rand_pos);
	ResetAgentTarObjRecord();
}

void cSceneKickBallAMP::UpdateTarObjs(double timestep)
{
	mTarObjTimer.Update(timestep);

	if (EnableRandTargetPos())
	{
		bool reset_objs = mTarObjTimer.IsEnd();
		if (reset_objs)
		{
			ResetTarObjs();
		}
	}
}

bool cSceneKickBallAMP::CheckTargetSucc() const
{
	const tVector ball_pos = GetBallPos();
	const tVector& tar_pos = GetTargetPos();
	tVector ball_tar_delta = tar_pos - ball_pos;
	ball_tar_delta[1] = 0;
	double ball_tar_dist_sq = ball_tar_delta.squaredNorm();
	bool term = ball_tar_dist_sq < mTargetSuccDist* mTargetSuccDist;
	return term;
}

cSceneKickBallAMP::eTerminate cSceneKickBallAMP::CheckTerminateTarget(int agent_id) const
{
	eTerminate terminated = cSceneTargetAMP::CheckTerminateTarget(agent_id);
	if (terminated == eTerminateNull)
	{
		if(mIsFail) terminated = eTerminateFail;
		else
		{
			bool term_succ = CheckTargetSucc();
			terminated = (term_succ) ? eTerminateSucc : terminated;

			if (terminated != eTerminateSucc && mMarkHit && (mTimer.GetTime() - mMarkHitTime) > mGoalLimitTime)
			{
				terminated = eTerminateFail;
			}
		}

	}
	return terminated;
}

bool cSceneKickBallAMP::CheckMarkSucc()
{
	assert(mStrikeBodies.size() > 0);
	tVector target_pos = GetBallPos();

	for (int i = 0; i < GetNumChars(); ++i)
	{
		const auto& sim_char = GetCharacter(i);

		tVector root_pos = sim_char->GetRootPos();
		tVector tar_root_delta = target_pos - root_pos;
		tar_root_delta[1] = 0.0;
		double norm = tar_root_delta.norm();

		tVector tar_root_dir = tVector::Zero();
		if (norm > 1e-5)
		{
			tar_root_dir = tar_root_delta / norm;
		}

		for (int j = 0; j < GetNumStrikeBodies(); ++j)
		{
			int b = mStrikeBodies[j];
			const auto& body_part = sim_char->GetBodyPart(b);
			const tVector pos = body_part->GetPos();
			double dist_sq = (target_pos - pos).squaredNorm();

			if (dist_sq < (mBallRadius+0.1) * (mBallRadius+0.1) )
			{
				const tVector vel = body_part->GetLinearVelocity();
				double speed = tar_root_dir.dot(vel);

				double tar_speed = mHitTarSpeed;
				if ((speed >= tar_speed) || (tar_speed == 0.0))
				{
					return true;
				}
				else
				{
					mIsFail = true;
				}
			}
		}
	}

	return false;

}

bool cSceneKickBallAMP::CheckCanKick() const
{

	tVector target_pos = GetTargetPos();
	tVector ball_pos = GetBallPos();

	for (int i = 0; i < GetNumChars(); ++i)
	{
		const auto& sim_char = GetCharacter(i);

		tVector root_pos = sim_char->GetRootPos();
		tVector tar_root_delta = target_pos - root_pos;

		double near_dist = GetTarNearDist();

		tVector ball_root_delta = ball_pos - root_pos;

		double tar_root_dist_sq = tar_root_delta[0] * tar_root_delta[0] + tar_root_delta[2] * tar_root_delta[2];
		double ball_root_dist_sq = ball_root_delta[0] * ball_root_delta[0] + ball_root_delta[2] * ball_root_delta[2];

		if(tar_root_dist_sq < mCanKickDist * mCanKickDist && ball_root_dist_sq < near_dist * near_dist)
			return true;
	}

	return false;
}

double cSceneKickBallAMP::GetTarObjMinDist() const
{
	return mMinTarObjDist;
}

double cSceneKickBallAMP::GetTarObjMaxDist() const
{
	return mMaxTarObjDist;
}

void cSceneKickBallAMP::SetTarObjPos(int obj_id, const tVector& pos)
{
	const auto& ball = GetObj(obj_id);
	double r = GetBallRadius();

	tVector ground_pos = pos;
	ground_pos[1] = r + mGround->SampleHeight(ground_pos);

	tVector rot_axis = tVector(mRand.RandDouble(-1, 1), mRand.RandDouble(-1, 1), mRand.RandDouble(-1, 1), 0).normalized();
	double rot_theta = cMathUtil::RandDouble(-M_PI, M_PI);

	ball->SetPos(ground_pos);
	ball->SetRotation(rot_axis, rot_theta);
	ball->SetLinearVelocity(tVector::Zero());
	ball->SetAngularVelocity(tVector::Zero());

	mKickTargetPos = ground_pos;
	mKickTargetPos[1] = ground_pos[1] + r;
	
	mMarkHit = false;
}

tVector cSceneKickBallAMP::SampleRandTargetPos()
{
	const double max_dist = GetMaxTargetDist();
	const auto& character = GetCharacter();

	const tVector ball_pos = GetBallPos();
	tVector target_pos = tVector::Zero();

	double r = mRand.RandDouble(GetBallRadius(), max_dist);
	if(r < 3) r = 3;
	double theta = mRand.RandDouble(-M_PI, M_PI);
	target_pos[0] = ball_pos[0] + r * std::cos(theta);
	target_pos[2] = ball_pos[2] + r * std::sin(theta);

	return target_pos;
}

void cSceneKickBallAMP::RecordCtrlState(int agent_id, Eigen::VectorXd& out_state) const
{
	cSceneTargetAMP::RecordState(agent_id, out_state);
}

int cSceneKickBallAMP::GetCtrlStateSize(int agent_id) const
{
	return cSceneTargetAMP::GetStateSize(agent_id);
}

void cSceneKickBallAMP::BuildCtrlStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cSceneTargetAMP::BuildStateOffsetScale(agent_id, out_offset, out_scale);
}

void cSceneKickBallAMP::BuildCtrlStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	cSceneTargetAMP::BuildStateNormGroups(agent_id, out_groups);
}

int cSceneKickBallAMP::GetTaskStateSize(int agent_id) const
{
	int s_size = 3 * cKinTree::gPosDim + cKinTree::gVelDim + cKinTree::gAngVelDim;
	return s_size;
}

void cSceneKickBallAMP::BuildTaskStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int s_size = GetTaskStateSize(agent_id);
	out_offset = Eigen::VectorXd::Zero(s_size);
	out_scale = Eigen::VectorXd::Ones(s_size);
}

void cSceneKickBallAMP::RecordTaskState(int agent_id, Eigen::VectorXd& out_state) const
{
	int s_size = GetTaskStateSize(agent_id);
	out_state = std::numeric_limits<double>::quiet_NaN() * Eigen::VectorXd::Ones(s_size);

	const cSimCharacter* sim_char = GetAgentChar(agent_id);
	tMatrix origin_trans = sim_char->BuildOriginTrans();
	tQuaternion origin_quat = cMathUtil::RotMatToQuaternion(origin_trans);

	const auto& obj = GetObj(GetTarObjID());
	tVector pos = obj->GetPos();
	tQuaternion rot = obj->GetRotation();
	tVector vel = obj->GetLinearVelocity();
	tVector ang_vel = obj->GetAngularVelocity();
	double ground_h = mGround->SampleHeight(pos);

	pos[1] -= ground_h;
	pos[3] = 1;
	pos = origin_trans * pos;
	pos[3] = 0;

	rot = origin_quat * rot;
	tVector rot_norm;
	tVector rot_tan;
	cMathUtil::CalcNormalTangent(rot, rot_norm, rot_tan);

	vel = cMathUtil::QuatRotVec(origin_quat, vel);
	ang_vel = cMathUtil::QuatRotVec(origin_quat, ang_vel);

	out_state.segment(0, cKinTree::gPosDim) = pos.segment(0, cKinTree::gPosDim);
	out_state.segment(cKinTree::gPosDim, cKinTree::gPosDim) = rot_norm.segment(0, cKinTree::gPosDim);
	out_state.segment(2 * cKinTree::gPosDim, cKinTree::gPosDim) = rot_tan.segment(0, cKinTree::gPosDim);
	out_state.segment(3 * cKinTree::gPosDim, cKinTree::gVelDim) = vel.segment(0, cKinTree::gVelDim);
	out_state.segment(3 * cKinTree::gPosDim + cKinTree::gVelDim, cKinTree::gAngVelDim) = ang_vel.segment(0, cKinTree::gAngVelDim);
}

void cSceneKickBallAMP::BuildTaskStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	int s_size = GetTaskStateSize(agent_id);
	out_groups = cCharController::gNormGroupSingle * Eigen::VectorXi::Ones(s_size);
}

int cSceneKickBallAMP::GetNumStrikeBodies() const
{
	return static_cast<int>(mStrikeBodies.size());
}

bool cSceneKickBallAMP::GoalIn() const
{
	return mIsGoal;
}

double cSceneKickBallAMP::GetTarNearDist() const
{
	return mTarNearDist;
}

bool cSceneKickBallAMP::GetMarkHit() const
{
	return mMarkHit;
}

double cSceneKickBallAMP::GetCanKickDist() const
{
	return mCanKickDist;
}

void cSceneKickBallAMP::UpdateTarget(double timestep)
{
	cSceneTargetAMP::UpdateTarget(timestep);


	if (!mMarkHit && CheckCanKick())
	{
		bool mark_hit = CheckMarkSucc();
		mMarkHit = mark_hit;
		if (mark_hit)
		{
			mMarkHitTime = mTimer.GetTime();
		}
	}
}
