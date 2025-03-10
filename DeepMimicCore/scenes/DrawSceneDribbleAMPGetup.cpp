#include "DrawSceneDribbleAMPGetup.h"
#include "SceneDribbleAMPGetup.h"
#include "render/DrawUtil.h"
#include "render/DrawObj.h"

cDrawSceneDribbleAMPGetup::cDrawSceneDribbleAMPGetup()
{
}

cDrawSceneDribbleAMPGetup::~cDrawSceneDribbleAMPGetup()
{
}

void cDrawSceneDribbleAMPGetup::BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cSceneDribbleAMPGetup>(new cSceneDribbleAMPGetup());
}

void cDrawSceneDribbleAMPGetup::HandleRayTest(const cWorld::tRayTestResult& result)
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

void cDrawSceneDribbleAMPGetup::SetBallPos(const tVector& pos)
{
	auto dribble_scene = dynamic_cast<cSceneDribbleAMPGetup*>(mScene.get());
	dribble_scene->SetBallPos(pos);
}

bool cDrawSceneDribbleAMPGetup::LoadTextures()
{
	bool succ = cDrawSceneTargetAMP::LoadTextures();
	mBallTex = std::unique_ptr<cTextureDesc>(new cTextureDesc("data/textures/soccer_ball.png", true));
	succ &= mBallTex->IsValid();
	return succ;
}

int cDrawSceneDribbleAMPGetup::GetTarObjID() const
{
	auto dribble_scene = dynamic_cast<cSceneDribbleAMPGetup*>(mScene.get());
	return dribble_scene->GetTarObjID();
}
void cDrawSceneDribbleAMPGetup::DrawObj(int obj_id) const
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
void cDrawSceneDribbleAMPGetup::DrawBall(int obj_id) const
{
	const double roughness = 0.4;
	const double enable_tex = 1;

	const cSceneSimChar::tObjEntry& entry = mScene->GetObjEntry(obj_id);
	if (entry.IsValid())
	{
		const auto& obj = entry.mObj;

		mShaderMesh->SetUniform4(mMeshMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
		mBallTex->BindTex(GL_TEXTURE0);

		cDrawUtil::SetColor(entry.mColor);
		cDrawObj::Draw(obj.get(), cDrawUtil::eDrawSolid);

		mBallTex->UnbindTex(GL_TEXTURE0);

		mShaderMesh->SetUniform4(mMeshMaterialDataHandle, tVector(roughness, 0, 0, 0));
	}
}