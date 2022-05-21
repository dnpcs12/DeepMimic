#include "SceneDribbleAMPGetup.h"
#include "anim/ClipsController.h"
#include "sim/SimSphere.h"

#define M_PI 3.14159265358979323846

const double cSceneDribbleAMPGetup::gComBallDistThreshold = 2;

double cSceneDribbleAMPGetup::CalcReward(int agent_id) const
{
	double reward = 0.0;
	bool getting_up = CheckGettingUp();
	if (getting_up)
	{
		reward = CalcRewardGetup(agent_id);
	}
	else
	{
		//r = cSceneHeadingAMP::CalcReward(agent_id);

		if (mMode == eModeTest)
		{
			reward = CalcRewardTest(agent_id);
		}
		else
		{
			reward = CalcRewardTrain(agent_id);
		}
	}
	return reward;
}

double cSceneDribbleAMPGetup::CalcRewardGetup(int agent_id) const
{
	const auto* sim_char = GetAgentChar(agent_id);
	tVector root_pos = sim_char->GetRootPos();
	tVector head_pos = sim_char->GetBodyPartPos(mHeadID);
	const auto& ground = GetGround();
	double ground_h = ground->SampleHeight(root_pos);

	double root_h = root_pos[1] - ground_h;
	double norm_root_h = root_h / mGetupHeightRoot;
	norm_root_h = cMathUtil::Clamp(norm_root_h, 0.0, 1.0);

	double head_h = head_pos[1] - ground_h;
	double norm_head_h = head_h / mGetupHeightHead;
	norm_head_h = cMathUtil::Clamp(norm_head_h, 0.0, 1.0);

	double r = 0.2 * norm_root_h + 0.8 * norm_head_h;

	return r;
}

void cSceneDribbleAMPGetup::ResetRecoveryEpisode()
{
	ResetTimers();
	BeginGetup();

	int num_chars = GetNumChars();
	for (int c = 0; c < num_chars; ++c)
	{
		const auto& sim_char = GetCharacter(c);
		const auto& ctrl = sim_char->GetController();
		if (ctrl != nullptr)
		{
			ctrl->Reset();
		}
	}
}

cSceneDribbleAMPGetup::cSceneDribbleAMPGetup() : cSceneHeadingAMP()
{
	mGetupTime = 0.0;
	mGetupHeightRoot = 0.5;
	mGetupHeightHead = 0.5;
	mHeadID = 0;

	mRecoverEpisodeProb = 0.0;
	mIsRecoveryEpisode = false;


	// dribble AMP below
	mTargetTimerParams.mTimeMin = 50;
	mTargetTimerParams.mTimeMax = 100;
	mTarObjTimerParams.mTimeMin = 100;
	mTarObjTimerParams.mTimeMax = 200;
	mMinTarObjDist = 0.5;
	mMaxTarObjDist = 10;

	mBallRadius = 0.2;
}

cSceneDribbleAMPGetup::~cSceneDribbleAMPGetup()
{
}

void cSceneDribbleAMPGetup::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cSceneHeadingAMP::ParseArgs(parser);

	parser->ParseInts("getup_motion_ids", mGetupMotionIDs);
	parser->ParseDouble("getup_height_root", mGetupHeightRoot);
	parser->ParseDouble("getup_height_head", mGetupHeightHead);
	parser->ParseInt("head_id", mHeadID);
	parser->ParseDouble("recover_episode_prob", mRecoverEpisodeProb);

	// scene dribble amp

	parser->ParseDouble("rand_tar_obj_time_min", mTarObjTimerParams.mTimeMin);
	parser->ParseDouble("rand_tar_obj_time_max", mTarObjTimerParams.mTimeMax);
	parser->ParseDouble("min_tar_obj_dist", mMinTarObjDist);
	parser->ParseDouble("max_tar_obj_dist", mMaxTarObjDist);

	parser->ParseDouble("ball_radius", mBallRadius);
}

void cSceneDribbleAMPGetup::Init()
{
	cSceneHeadingAMP::Init();

	RecordGetupMotionFlags(mGetupMotionIDs);
	mGetupTime = CalcGetupTime(mGetupMotionIDs);

	InitGetupTimer();
	ResetGetupTimer();
	SyncGetupTimer();

	// copy from scene dribble AMP below

	//cSceneTargetAMP::Init();

	InitTarObjs();
	InitAgentTarObjRecord();

	mTargetTimer.Reset();
	ResetTarget();
}

void cSceneDribbleAMPGetup::Update(double timestep)
{
	cSceneHeadingAMP::Update(timestep);

	if (mMode == eModeTest)
	{
		UpdateTestGetup();
	}
}

void cSceneDribbleAMPGetup::Reset()
{
	bool mIsRecoveryEpisode = ActivateRecoveryEpisode();
	if (mIsRecoveryEpisode)
	{
		ResetRecoveryEpisode();
	}
	else
	{
		cSceneHeadingAMP::Reset();
		SyncGetupTimer();

		// copy from scene dribble AMP below

		mTarObjTimer.Reset();
		ResetTarObjs();
		ResetAgentTarObjRecord();

		cSceneTargetAMP::Reset();
	}

	
}

void cSceneDribbleAMPGetup::RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const
{
	cSceneHeadingAMP::RecordGoal(agent_id, out_goal);

	double getup_phase = CalcGetupPhase();
	int offset = cSceneHeadingAMP::GetGoalSize(agent_id);
	out_goal[offset] = getup_phase;
}

int cSceneDribbleAMPGetup::GetGoalSize(int agent_id) const
{
	int g_size = cSceneHeadingAMP::GetGoalSize(agent_id);
	g_size += 1;
	return g_size;
}

void cSceneDribbleAMPGetup::BuildGoalOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cSceneHeadingAMP::BuildGoalOffsetScale(agent_id, out_offset, out_scale);

	int offset = cSceneHeadingAMP::GetGoalSize(agent_id);
	out_offset[offset] = -0.5;
	out_scale[offset] = 2.0;
}

void cSceneDribbleAMPGetup::BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	cSceneHeadingAMP::BuildGoalNormGroups(agent_id, out_groups);

	int offset = cSceneHeadingAMP::GetGoalSize(agent_id);
	out_groups[offset] = cCharController::gNormGroupNone;
}

std::string cSceneDribbleAMPGetup::GetName() const
{
	return "Dribble AMP Getup";
}

void cSceneDribbleAMPGetup::ResetTimers()
{
	cSceneHeadingAMP::ResetTimers();
	ResetGetupTimer();
}

void cSceneDribbleAMPGetup::UpdateTimers(double timestep)
{
	cSceneHeadingAMP::UpdateTimers(timestep);
	UpdateGetupTimer(timestep);
}

void cSceneDribbleAMPGetup::InitGetupTimer()
{
	mIsRecoveryEpisode = false;

	cTimer::tParams getup_timer_params;
	getup_timer_params.mType = cTimer::eTypeUniform;
	getup_timer_params.mTimeMin = mGetupTime;
	getup_timer_params.mTimeMax = mGetupTime;

	mGetupTimer.Init(getup_timer_params);
}

void cSceneDribbleAMPGetup::ResetGetupTimer()
{
	mGetupTimer.Reset();
	EndGetup();
}

void cSceneDribbleAMPGetup::SyncGetupTimer()
{
	const auto& kin_char = GetKinChar();
	const auto kin_ctrl = kin_char->GetController();
	const cClipsController* clips_ctrl = dynamic_cast<const cClipsController*>(kin_ctrl.get());
	if (clips_ctrl != nullptr)
	{
		int curr_motion_id = clips_ctrl->GetCurrMotionID();
		bool is_getup_motion = mGetupMotionFlags[curr_motion_id];

		if (is_getup_motion)
		{
			double kin_time = clips_ctrl->GetTime();
			mGetupTimer.SetTime(kin_time);
		}
	}
}

void cSceneDribbleAMPGetup::UpdateGetupTimer(double timestep)
{
	mGetupTimer.Update(timestep);
}

void cSceneDribbleAMPGetup::RecordGetupMotionFlags(const std::vector<int>& getup_motion_ids)
{
	const auto& kin_char = GetKinChar();
	const auto kin_ctrl = kin_char->GetController();
	const cClipsController* clips_ctrl = dynamic_cast<const cClipsController*>(kin_ctrl.get());

	if (clips_ctrl != nullptr)
	{
		int num_motions = clips_ctrl->GetNumMotions();
		mGetupMotionFlags.resize(num_motions, false);

		int num_getup_motions = static_cast<int>(getup_motion_ids.size());
		for (int i = 0; i < num_getup_motions; ++i)
		{
			int curr_id = getup_motion_ids[i];
			mGetupMotionFlags[curr_id] = true;
		}
	}
	else
	{
		assert(false), "Unsupported kin motion controller.";
	}
}

void cSceneDribbleAMPGetup::BeginGetup()
{
	mGetupTimer.SetTime(0.0);
}

void cSceneDribbleAMPGetup::EndGetup()
{
	mGetupTimer.SetTime(mGetupTime);
}

void cSceneDribbleAMPGetup::UpdateTestGetup()
{
	const auto& sim_char = GetCharacter();
	bool fallen = HasFallenContact(*sim_char);
	bool is_getup = CheckGettingUp();
	if (fallen && !is_getup)
	{
		BeginGetup();
	}
}

bool cSceneDribbleAMPGetup::HasFallenContact(const cSimCharacter& sim_char) const
{
	bool fallen = false;
	bool getting_up = CheckGettingUp();
	if (!getting_up)
	{
		fallen = cSceneHeadingAMP::HasFallenContact(sim_char);
	}
	return fallen;
}

double cSceneDribbleAMPGetup::CalcGetupTime(const std::vector<int>& getup_motion_ids) const
{
	double getup_time = 0.0;
	const auto& kin_char = GetKinChar();

	const auto kin_ctrl = kin_char->GetController();
	const cClipsController* clips_ctrl = dynamic_cast<const cClipsController*>(kin_ctrl.get());

	if (clips_ctrl != nullptr)
	{
		int num_getup_motions = static_cast<int>(getup_motion_ids.size());
		for (int i = 0; i < num_getup_motions; ++i)
		{
			int motion_id = getup_motion_ids[i];
			const cMotion& curr_motion = clips_ctrl->GetMotion(motion_id);
			double curr_dur = curr_motion.GetDuration();
			getup_time = std::max(curr_dur, getup_time);
		}
	}
	else
	{
		assert(false), "Unsupported kin motion controller.";
	}
	return getup_time;
}

double cSceneDribbleAMPGetup::CalcGetupPhase() const
{
	double getup_time = mGetupTimer.GetTime();
	double phase = 1.0 - getup_time / mGetupTime;
	phase = cMathUtil::Clamp(phase, 0.0, 1.0);
	return phase;
}

bool cSceneDribbleAMPGetup::CheckGettingUp() const
{
	return !mGetupTimer.IsEnd();
}

bool cSceneDribbleAMPGetup::ActivateRecoveryEpisode()
{
	bool activate = false;

	if ((mMode == eModeTrain) && (mRecoverEpisodeProb > 0.0) && !mIsRecoveryEpisode)
	{
		eTerminate terminate = CheckTerminate(0);
		if (terminate == eTerminateFail)
		{
			activate = mRand.FlipCoin(mRecoverEpisodeProb);
		}
	}
	return activate;
}

void cSceneDribbleAMPGetup::SetBallPos(const tVector& pos)
{
	SetTarObjPos(GetTarObjID(), pos);
	ResetAgentTarObjRecord();
}
tVector cSceneDribbleAMPGetup::GetBallPos() const
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
int cSceneDribbleAMPGetup::GetTarObjID() const
{
	return mTarObjID;
}

void cSceneDribbleAMPGetup::RecordState(int agent_id, Eigen::VectorXd& out_state) const
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
int cSceneDribbleAMPGetup::GetStateSize(int agent_id) const
{
	int ctrl_s_size = GetCtrlStateSize(agent_id);
	int task_s_size = GetTaskStateSize(agent_id);
	int s_size = ctrl_s_size + task_s_size;
	return s_size;
}
void cSceneDribbleAMPGetup::BuildStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
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
void cSceneDribbleAMPGetup::BuildStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
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

// copy from scene dribble AMP below

void cSceneDribbleAMPGetup::InitAgentTarObjRecord()
{
	int num_agents = GetNumAgents();
	mAgentPrevTarObjPos.resize(num_agents);
	ResetAgentTarObjRecord();
}
void cSceneDribbleAMPGetup::ResetAgentTarObjRecord()
{
	for (size_t i = 0; i < mAgentPrevTarObjPos.size(); ++i)
	{
		UpdateAgentTarObjRecord(i);
	}
}
void cSceneDribbleAMPGetup::UpdateAgentTarObjRecord(int agent_id)
{
	tVector ball_pos = GetBallPos();
	mAgentPrevTarObjPos[agent_id] = ball_pos;
}
double cSceneDribbleAMPGetup::GetBallRadius() const
{
	return mBallRadius;
}

void cSceneDribbleAMPGetup::NewActionUpdate(int agent_id)
{
	cSceneTargetAMP::NewActionUpdate(agent_id);
	UpdateAgentTarObjRecord(agent_id);
}
bool cSceneDribbleAMPGetup::HasFallen(const cSimCharacter& sim_char) const
{
	bool fallen = cSceneTargetAMP::HasFallen(sim_char);
	fallen |= CheckTarObjDistFail();
	fallen |= CheckCharObjDistFail(sim_char);
	return fallen;
}
bool cSceneDribbleAMPGetup::CheckTarObjDistFail() const
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
bool cSceneDribbleAMPGetup::CheckCharObjDistFail(const cSimCharacter& sim_char) const
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

void cSceneDribbleAMPGetup::UpdateObjs(double timestep)
{
	cSceneTargetAMP::UpdateObjs(timestep);

	UpdateTarObjs(timestep);
	if (mTarObjTimer.IsEnd())
	{
		mTarObjTimer.Reset();
	}
}
void cSceneDribbleAMPGetup::ClearObjs()
{
	cSceneTargetAMP::ClearObjs();
	mTarObjID = gInvalidIdx;
}

double cSceneDribbleAMPGetup::CalcRewardTrain(int agent_id) const
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
	bool fallen = HasFallen(*character);
	if (!fallen)
	{
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
	}

	return reward;
}
double cSceneDribbleAMPGetup::CalcRewardTest(int agent_id) const
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

void cSceneDribbleAMPGetup::InitTarObjs()
{
	mTarObjTimer.Init(mTarObjTimerParams);
	BuildTarObjs();
	ResetTarObjs();
}
void cSceneDribbleAMPGetup::BuildTarObjs()
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
void cSceneDribbleAMPGetup::ResetTarObjs()
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
void cSceneDribbleAMPGetup::UpdateTarObjs(double timestep)
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
bool cSceneDribbleAMPGetup::CheckTargetSucc() const
{
	const tVector ball_pos = GetBallPos();
	const tVector& tar_pos = GetTargetPos();
	tVector ball_tar_delta = tar_pos - ball_pos;
	ball_tar_delta[1] = 0;
	double ball_tar_dist_sq = ball_tar_delta.squaredNorm();
	bool term = ball_tar_dist_sq < mTargetSuccDist* mTargetSuccDist;

	return term;
}
cSceneDribbleAMPGetup::eTerminate cSceneDribbleAMPGetup::CheckTerminateTarget(int agent_id) const
{
	eTerminate terminated = cSceneTargetAMP::CheckTerminateTarget(agent_id);
	if (terminated == eTerminateNull)
	{
		bool term_succ = CheckTargetSucc();
		terminated = (term_succ) ? eTerminateSucc : terminated;
	}
	return terminated;
}

double cSceneDribbleAMPGetup::GetTarObjMinDist() const
{
	return mMinTarObjDist;
}
double cSceneDribbleAMPGetup::GetTarObjMaxDist() const
{
	return mMaxTarObjDist;
}
void cSceneDribbleAMPGetup::SetTarObjPos(int obj_id, const tVector& pos)
{
	// set ball pos
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
}

tVector cSceneDribbleAMPGetup::SampleRandTargetPos()
{
	const double max_dist = GetMaxTargetDist();
	const auto& character = GetCharacter();

	const tVector ball_pos = GetBallPos();
	tVector target_pos = tVector::Zero();

	double r = mRand.RandDouble(GetBallRadius(), max_dist);
	double theta = mRand.RandDouble(-M_PI, M_PI);
	target_pos[0] = ball_pos[0] + r * std::cos(theta);
	target_pos[2] = ball_pos[2] + r * std::sin(theta);

	return target_pos;
}

void cSceneDribbleAMPGetup::RecordCtrlState(int agent_id, Eigen::VectorXd& out_state) const
{
	cSceneTargetAMP::RecordState(agent_id, out_state);
}
int cSceneDribbleAMPGetup::GetCtrlStateSize(int agent_id) const
{
	return cSceneTargetAMP::GetStateSize(agent_id);
}
void cSceneDribbleAMPGetup::BuildCtrlStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cSceneTargetAMP::BuildStateOffsetScale(agent_id, out_offset, out_scale);
}
void cSceneDribbleAMPGetup::BuildCtrlStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	cSceneTargetAMP::BuildStateNormGroups(agent_id, out_groups);
}

void cSceneDribbleAMPGetup::RecordTaskState(int agent_id, Eigen::VectorXd& out_state) const
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
int cSceneDribbleAMPGetup::GetTaskStateSize(int agent_id) const
{
	int s_size = 3 * cKinTree::gPosDim + cKinTree::gVelDim + cKinTree::gAngVelDim;
	return s_size;
}
void cSceneDribbleAMPGetup::BuildTaskStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int s_size = GetTaskStateSize(agent_id);
	out_offset = Eigen::VectorXd::Zero(s_size);
	out_scale = Eigen::VectorXd::Ones(s_size);
}
void cSceneDribbleAMPGetup::BuildTaskStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	int s_size = GetTaskStateSize(agent_id);
	out_groups = cCharController::gNormGroupSingle * Eigen::VectorXi::Ones(s_size);
}