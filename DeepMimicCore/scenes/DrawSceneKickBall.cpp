#include "DrawSceneKickBallAMP.h"
#include "cSceneKickBallAMP.h"
#include "render/DrawUtil.h"
#include "render/DrawObj.h"

cDrawSceneKickBallAMP::cDrawSceneKickBallAMP()
{
}

cDrawSceneKickBallAMP::~cDrawSceneKickBallAMP()
{
}

void cDrawSceneKickBallAMP::BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cSceneKickBallAMP>(new cSceneKickBallAMP());
}

void cDrawSceneKickBallAMP::HandleRayTest(const cWorld::tRayTestResult& result)
{
	bool handled = false;
	if (result.mObj != nullptr)
	{
		cSimObj::eType obj_type = result.mObj->GetType();
		if (obj_type == cSimObj::eTypeStatic && glutGetModifiers() == GLUT_ACTIVE_CTRL)
		{
			SetBallPos(result.mHitPos);
			handled = true;
		}
	}

	if (!handled)
	{
		cDrawSceneTargetAMP::HandleRayTest(result);
	}
}

void cDrawSceneKickBallAMP::SetBallPos(const tVector& pos)
{
	auto dribble_scene = dynamic_cast<cSceneKickBallAMP*>(mScene.get());
	dribble_scene->SetBallPos(pos);
}

bool cDrawSceneKickBallAMP::LoadTextures()
{
	bool succ = cDrawSceneTargetAMP::LoadTextures();
	mBallTex = std::unique_ptr<cTextureDesc>(new cTextureDesc("data/textures/soccer_ball.png", true));
	succ &= mBallTex->IsValid();
	return succ;
}

int cDrawSceneKickBallAMP::GetTarObjID() const
{
	auto dribble_scene = dynamic_cast<cSceneKickBallAMP*>(mScene.get());
	return dribble_scene->GetTarObjID();
}

void cDrawSceneKickBallAMP::DrawObj(int obj_id) const
{
	if (obj_id == GetTarObjID())
	{
		DrawBall(obj_id);
	}
	else
	{
		cDrawSceneTargetAMP::DrawObj(obj_id);
	}
}

void cDrawSceneKickBallAMP::DrawBall(int obj_id) const
{
	const double roughness = 0.4;
	const double enable_tex = 1;

	const cSceneSimChar::tObjEntry& entry = mScene->GetObjEntry(obj_id);

	auto scene = dynamic_cast<const cSceneKickBallAMP*>(mScene.get());
	bool hit = scene->GetMarkHit();

	if (entry.IsValid())
	{
		const auto& obj = entry.mObj;

		mShaderMesh->SetUniform4(mMeshMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
		mBallTex->BindTex(GL_TEXTURE0);

		tVector color = tVector(0, 0.75, 0, 0.5);
		if(!hit)
			color = entry.mColor;

		cDrawUtil::SetColor(color);
		
		cDrawObj::Draw(obj.get(), cDrawUtil::eDrawSolid);

		mBallTex->UnbindTex(GL_TEXTURE0);

		mShaderMesh->SetUniform4(mMeshMaterialDataHandle, tVector(roughness, 0, 0, 0));
	}
}

void cDrawSceneKickBallAMP::DrawTargetPos(const tVector& target_pos) const
{

	const tVector no_hit_col = tVector(1, 0, 0, 0.5);

	 cDrawSceneTargetAMP::DrawTargetPos(target_pos);

	 auto scene = dynamic_cast<const cSceneKickBallAMP*>(mScene.get());

	 const auto& ground = scene->GetGround();
	 double ground_h = ground->SampleHeight(target_pos);
	 const double dist = scene->GetCanKickDist();

	 cDrawUtil::PushMatrixView();

	 cDrawUtil::SetLineWidth(3);
	 cDrawUtil::SetColor(no_hit_col);
	 cDrawUtil::Translate(tVector(target_pos[0], ground_h + 0.1, target_pos[2], 0));
	 cDrawUtil::Rotate(0.5 * M_PI, tVector(1, 0, 0, 0));
	 cDrawUtil::DrawDisk(dist, cDrawUtil::eDrawWireSimple);

	 cDrawUtil::PopMatrixView();
}
