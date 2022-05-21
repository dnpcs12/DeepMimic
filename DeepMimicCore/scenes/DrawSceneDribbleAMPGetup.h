#pragma once

#include "scenes/DrawSceneHeadingAMP.h"

class cDrawSceneDribbleAMPGetup : virtual public cDrawSceneHeadingAMP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		cDrawSceneDribbleAMPGetup();
	virtual ~cDrawSceneDribbleAMPGetup();

protected:

	virtual void BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const;

	// copy from draw scene dribble AMP below

	std::unique_ptr<cTextureDesc> mBallTex;

	//virtual void BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const;
	virtual void HandleRayTest(const cWorld::tRayTestResult& result);
	virtual void SetBallPos(const tVector& pos);

	virtual bool LoadTextures();

	virtual int GetTarObjID() const;
	virtual void DrawObj(int obj_id) const;
	virtual void DrawBall(int obj_id) const;
};
