#include "GameObjects.h"


BouncingBall::BouncingBall() : ParabolicBounceData()
{};


BouncingBall::~BouncingBall()
{
	ControllerManager::getSingleton().removeController(timeController);
};


void BouncingBall::setupBounceData(float rad, const Vector2 &pos0, float fallDist, float arenaWidth, float horizontalVelocity)
{
	ball = std::make_shared<Collider>();
	ball->addShape(std::move(std::make_unique<Circle>(Vector2(0.0f, 0.0f), rad)));

	radius = rad;
	x0 = pos0.x;
	height = pos0.y;
	length = fallDist;
	width = arenaWidth;
	velocity = horizontalVelocity;
	time = 0.0f;

	progressTime(0.0f);

	SharedControllerValuePtr ptr = std::make_shared<TTimeframeControllerValue<BouncingBall>>(this);
	timeController = ControllerManager::getSingleton().createFrameTimeController(ptr);
};


void BouncingBall::progressTime(float frameTime)
{
	time += frameTime;
	PositionalData pos = parabolicBounce(*this);
	ball->setPosition(pos.position);
	ball->setVelocity(pos.movementDirection * velocity);

	if (pos.movementDirection.y < 0.0f)
		pos.movementDirection = -pos.movementDirection;

	//if (std::abs(pos.movementDirection.y / pos.movementDirection.x) >= 2.0f)
		//pos.movementDirection.x = -pos.movementDirection.x;

	float rot = std::fmod((float) (2.0*M_PI) + std::atan2(pos.movementDirection.y, pos.movementDirection.x) + (float) (3.0*M_PI_2), (float) (2.0*M_PI));

	constexpr double modAngle = M_PI * 0.15;

	if (rot > (float)(modAngle) && rot < (float)(2.0*M_PI - modAngle))
	{
		if (rot <= M_PI)
		{
			float diff = (((float)(M_PI_2 - modAngle) - (rot - (float)(modAngle))) / ((float)(M_PI_2 - modAngle)))*(float)(modAngle);
			rot = diff;
		}
		else
		{
			float diff = (((float)(3.0*M_PI_2 + modAngle) - (rot + (float)(modAngle))) / ((float)(M_PI_2 - modAngle)))*(float)(modAngle);
			rot = (float)(2.0*M_PI) + diff;
		}


		std::wstring text(1000, '\0');
		swprintf_s(text.data(), 1000, L"dir: %f, %f \t\t rot: %f \r\n", pos.movementDirection.x, pos.movementDirection.y, rot);
		OutputDebugString(text.c_str());

/*
		if (rot <= M_PI)
			rot = (float)(M_PI_2)-rot;
		else
			//rot = rot;
			rot = (float)(2.0 * M_PI- M_PI_2)-rot;
*/
	}
		
		
		
		
		
		
		
		//|| rot < (float)(2.0*M_PI -M_PI_4))


	ball->setPosition(Quaternion(rot));


	std::wstring text(1000, '\0');
	swprintf_s(text.data(), 1000, L"dir: %f, %f \t\t rot: %f \r\n", pos.movementDirection.x, pos.movementDirection.y, rot);
	//OutputDebugString(text.c_str());

};


void BouncingBall::render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght) const
{
	ball->render(_pDevice, _vBuffer, _bufferLenght, 0xFFE10000);

	/*
	Vector2 pos = ball->getPosition();
	Vector2 dir2 = Quaternion(std::fmod((float) 2.0f*M_PI + std::atan2(dir.y, dir.x), (float) 2.0f*M_PI)) * Vector2(1.0f, 0.0f);

	struct VertStruct {
		Vector2 xy;
		float z;
		DWORD color;
	} *vertStruct;
	_vBuffer->Lock(0, 0, (void**)&vertStruct, D3DLOCK_DISCARD);
	vertStruct[0] = { pos, 0.0f, 0xFFFFFFFF };
	vertStruct[1] = { pos + 50.0f*dir2, 0.0f, 0xFFFFFFFF };

	_vBuffer->Unlock();
	_pDevice->SetStreamSource(0, _vBuffer, 0, sizeof(VertStruct));
	_pDevice->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE);
	_pDevice->DrawPrimitive(D3DPT_LINELIST, 0, 2);
	*/
};


FloatingBall::FloatingBall()
{};


FloatingBall::~FloatingBall()
{
	ControllerManager::getSingleton().removeController(timeController);
};


void FloatingBall::setupFloatData(float rad, const Vector2 &pos0, float waveLength, float waveAmplitude, float horizontalVelocity)
{
	ball = std::make_shared<Collider>();
	ball->addShape(std::move(std::make_unique<Circle>(Vector2(0.0f, 0.0f), rad)));

	x0 = pos0.x;
	length = waveLength;
	amplitude = waveAmplitude;
	height = pos0.y;
	velocity = horizontalVelocity;
	time = 0.0f;

	progressTime(0.0f);

	SharedControllerValuePtr ptr = std::make_shared<TTimeframeControllerValue<FloatingBall>>(this);
	timeController = ControllerManager::getSingleton().createFrameTimeController(ptr);
};


void FloatingBall::progressTime(float frameTime)
{
	time += frameTime;
	PositionalData pos = sinusoidalBounce(*this);
	ball->setPosition(pos.position);
	ball->setVelocity(pos.movementDirection * velocity);
	//ball->setRotation(Quaternion(std::fmod((float) 2.0f*M_PI + std::atan2(pos.movementDirection.y, pos.movementDirection.x), (float) 2.0f*M_PI)));
};


void FloatingBall::render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght) const
{
	ball->render(_pDevice, _vBuffer, _bufferLenght, 0xFFE10000);

	/*
	Vector2 pos = ball->getPosition();
	Vector2 dir2 = Quaternion(std::fmod((float) 2.0f*M_PI + std::atan2(dir.y, dir.x), (float) 2.0f*M_PI)) * Vector2(1.0f, 0.0f);

	struct VertStruct {
		Vector2 xy;
		float z;
		DWORD color;
	} *vertStruct;
	_vBuffer->Lock(0, 0, (void**)&vertStruct, D3DLOCK_DISCARD);
	vertStruct[0] = { pos, 0.0f, 0xFFFFFFFF };
	vertStruct[1] = { pos + 50.0f*dir2, 0.0f, 0xFFFFFFFF };

	_vBuffer->Unlock();
	_pDevice->SetStreamSource(0, _vBuffer, 0, sizeof(VertStruct));
	_pDevice->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE);
	_pDevice->DrawPrimitive(D3DPT_LINELIST, 0, 2);
	*/
};
