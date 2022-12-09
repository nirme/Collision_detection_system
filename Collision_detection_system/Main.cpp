#include <iostream>
#include <windows.h>

//Direct3D 9 with helper api
#include <d3d9.h>
#pragma comment (lib, "d3d9.lib")

//DirectX8 Input
#define DIRECTINPUT_VERSION  0x0800
#include <dinput.h>
#pragma comment (lib, "dinput8.lib")
#pragma comment (lib, "dxguid.lib")

#include <random>
#include <time.h>
#include <chrono>
#include <memory>

#include "ImpulseStructs.h"
//#include "PhysicsBroadPhase.h"
#include "PhysicsNarrowPhase.h"
#include "ControllerManager.h"

#include "Circle.h"
#include "Ellipse.h"
#include "ConvexPolygon.h"
#include "Pill.h"
#include "PointMass.h"

#include "Constants.h"
#include <math.h>

//#include "TrackFunctions.h"
#include "GameObjects.h"



#define KEY_ADD DIK_COMMA
#define KEY_REMOVE DIK_PERIOD
#define KEY_EX1 DIK_1
#define KEY_EX2	DIK_2
#define KEY_SPACE DIK_SPACE
#define KEY_BACK DIK_BACKSPACE


PhysicsNarrowPhase *collisionPhysics;
//PhysicsBroadPhase *collisionPhysicsBroadPhase;

constexpr float size_multiplier = 2;



static LPDIRECTINPUT8 dInput;		// DirectInput interface
static LPDIRECTINPUTDEVICE8 dInputKeyboard;
static LPDIRECTINPUTDEVICE8 dInputMouse;
static BYTE keyState[256];		// key-info table
static BYTE keyStateOld[256];
static DIMOUSESTATE mouseState;		//mouse-info
static float mouseSpeed;

static const wchar_t *className = L"Test class";
static const wchar_t *windowName = L"Test app";

static int wWidth = 800;
static int wHeight = 600;
static int refreshRate = 60;

static HINSTANCE hInstance;
static HWND hWnd;
static WNDCLASSEX wndClassEx;

static IDirect3D9* pD3D9;
static IDirect3DDevice9* pDevice;
static D3DCAPS9 pDeviceCaps;

static D3DDISPLAYMODE mode;
//static DISPLAY_FORMAT displayFormat;

typedef std::chrono::time_point<std::chrono::high_resolution_clock> Timepoint;
Timepoint lastTime;
Timepoint currentTime;
float timer = 0.0f;
unsigned int secondsTimer = 0;


float viewScale = 20.0f;
int experiment = -1;


std::vector<std::string> colliders;



char* generateRandomString(char* s, unsigned int len)
{
	static const char *val = "qwertyuiopasdfghjklzxcvbnmQWERTYUIOPASDFGHJKLZXCVBNM1234567890\0";
	static const unsigned int size = (unsigned int)strlen(val);

	std::srand((unsigned int)time(nullptr));

	unsigned int i = 0;
	while (i < len)
	{
		s[i] = val[std::rand() % size];
		++i;
	}
	
	return s;
};




LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg)
	{
	case WM_DESTROY:
		PostQuitMessage(0);
		break;

//#ifdef _DEBUG
	case WM_KEYDOWN:
		if (wParam == VK_ESCAPE)
			DestroyWindow(hwnd);

		break;

//#endif
	}
	return DefWindowProc(hwnd, msg, wParam, lParam);
}


ColliderShapePtr createCircle(float sizeMax, float sizeMin)
{
	float radius = ((std::rand() % (int)((sizeMax - sizeMin) * 100.0f)) * 0.01f + sizeMin) * 0.5f;

	float maxDist = 20.0f;
	float rr = (std::rand() % 1000) * 0.001f;
	float l = (std::rand() % 1000) * 0.001f;

	Vector2 center = Quaternion((float)(2.0*M_PI_2)*rr) * Vector2(maxDist * l, 0.0f);

	/*
	Vector2 center(
		(std::rand() % (int)(width - (2.0f * radius))) - (width * 0.5f) + radius, 
		(std::rand() % (int)(height - (2.0f * radius))) - (height * 0.5f) + radius
	);
	*/
	return std::make_unique<Circle>(Vector2(0.0f, 0.0f), radius);
};


ColliderShapePtr createEllipse(float sizeMax, float sizeMin)
{
	float radius = ((std::rand() % (int)((sizeMax - sizeMin) * 100.0f)) * 0.01f + sizeMin);
	/*
	Vector2 center(
		(std::rand() % (int)(width - (2.0f * radius))) - (width * 0.5f) + radius,
		(std::rand() % (int)(height - (2.0f * radius))) - (height * 0.5f) + radius
	);
	*/
	float rot = rand() % 201 * 0.01f * (float)M_PI;
	float s = (rand() % 3 + 1.0f) * 0.25f;

	if (rand() % 2)
		return std::make_unique<class Ellipse>(Vector2(0.0f), radius, radius * s, rot);
	return std::make_unique<class Ellipse>(Vector2(0.0f), radius * s, radius, rot);
};

ColliderShapePtr createRectangle(float sizeMax, float sizeMin)
{
	Vector2 p(0.5f);

	float size = ((std::rand() % (int)((sizeMax - sizeMin) * 100.0f)) * 0.01f + sizeMin);
	p *= size;

	float ratio = (rand() % 8 + 1.0f) * 0.125f;
	p = rand() % 2 ? Vector2(p.x*ratio, p.y) : Vector2(p.x, p.y*ratio);

	float rot = rand() % 201 * 0.01f * (float)M_PI;
	Quaternion q(rot);

	p.x = std::abs(p.x);
	p.y = std::abs(p.y);
	
	Vector2 points[4] = { q*p, q*Vector2(-p.x,p.y), q*Vector2(-p.x,-p.y), q*Vector2(p.x,-p.y) };

	return std::make_unique<ConvexPolygon>(&points[0], 4);
};

ColliderShapePtr createPolygon(float sizeMax, float sizeMin)
{
	int pp = 30;
	std::vector<Vector2> points(pp);

	float spread = (sizeMax - sizeMin) * 0.5f, halfMin = sizeMin * 0.5f;
	for (int i = 0; i < pp; ++i)
	{
		points[i] = Vector2(
			((rand() % 10001) * 0.0001f * spread + halfMin) * (rand() % 2 ? 1 : -1),
			((rand() % 10001) * 0.0001f * spread + halfMin) * (rand() % 2 ? 1 : -1)
		);
	}

	std::vector<Vector2> hull = generateHull(points);

	return std::make_unique<ConvexPolygon>(&points[0], (unsigned int)points.size());
};

ColliderShapePtr createPill(float sizeMax, float sizeMin)
{
	float r = 15;// (sizeMax - sizeMin) * ((rand() % 20 + 1) * 0.01f);
	float l = (rand() % 101 * 0.01f) * (sizeMax - sizeMin - 2.0f * r) + sizeMin;
	float t = (rand() % 201 * 0.01f) * (float)M_PI;

	return std::make_unique<Pill>(Vector2(0.0f), l, r, t);
};




void addCollider()
{
	//Collider *c = new Collider(pos, rot, speed, angularSpeed);
	ColliderSPtr c = std::make_shared<Collider>();
	char name[COLLIDER_ID_LEN];
	c->setName(generateRandomString(name, COLLIDER_ID_LEN));
	float angularSpeed = rand() % 10 * 0.1f * (float)M_PI * (rand() % 2 ? 1.0f : -1.0f)*2.f;
	c->setVelocity(angularSpeed);

	ColliderShapePtr shape;
	//shape = createRectangle(40.0f * size_multiplier, 20.0f * size_multiplier);

	//switch (rand() % 4)
	switch (1)
	{
	case 0:
		shape = createCircle(50.0f * size_multiplier, 30.0f * size_multiplier);
		break;

	case 1:
		shape = createEllipse(40.0f * size_multiplier, 30.0f * size_multiplier);
		break;
		
	case 2:
		shape = createRectangle(40.0f * size_multiplier, 20.0f * size_multiplier);
		break;

	case 3:
		shape = createPill(60.0f * size_multiplier, 40.0f * size_multiplier);
		break;
	}

	shape->setDensity(0.01f);

	unsigned int val = (unsigned int)(((shape->getMass() - 1.0f) / 79.0f) * 0x80) + 0x7F;
	c->color = 0xFF000000 | val << 16 | val << 8 | val;

	c->addShape(std::move(shape));
	c->updatePhysicsParams();

	//c->color = 0xFF000000 | (0x00FF0000 & (std::rand() << 16)) | (0x0000FFFF & std::rand());

	do
	{
		Vector2 pos = { (float)(((rand() % 10000) * 0.0001f) * (wHeight / 2 - 20.0f)), 0.0f };
		pos = Quaternion(((rand() % 10000) * 0.0001f) * (2.0f * (float)M_PI)) * pos;
		Vector2 speed((rand() % 300 - 150) * 0.1f, (rand() % 300 - 150) * 0.1f);
		speed = -pos.normalize() * ((rand() % 1000) * 0.1f)*2.f;

		c->setPosition(pos);
		c->setPosition(Quaternion(0.0f));
		c->setVelocity(speed);
	}
	while (collisionPhysics->intersect(*c, true));


	//c->addAccelerator(collisionPhysics->getAccelerator("gravity"));

	collisionPhysics->addColider(c);
	colliders.push_back(name);
};

void removeCollider()
{
	if (!colliders.size())
		return;
	int num = std::rand() % colliders.size();
	auto it = colliders.begin();
	while (num--) ++it;

	collisionPhysics->removeCollider((*it).c_str());
	colliders.erase(it);
};


void runDistanceExperiment()
{
	collisionPhysics->showDistance = true;

	addCollider();
	addCollider();
};






int WINAPI WinMain(HINSTANCE hinstance, HINSTANCE prevInstance, PSTR cmdLine, int showCmd)
{
	srand((unsigned int)time(NULL));

	// start collision
	ControllerManager controllerMng;
	collisionPhysics = new PhysicsNarrowPhase();
	collisionPhysics->setBroadphaseMargin(10.0f);

	hInstance = GetModuleHandle(NULL);
	hWnd = 0;

	ZeroMemory(&wndClassEx, sizeof(WNDCLASSEX));
	wndClassEx.cbSize = sizeof(WNDCLASSEX);
	wndClassEx.style = CS_HREDRAW | CS_VREDRAW;
	wndClassEx.lpfnWndProc = WndProc;
	//	wndClassEx.cbClsExtra =		NULL;
	//	wndClassEx.cbWndExtra =		NULL;
	wndClassEx.hInstance = hInstance;
	//	wndClassEx.hIcon =			LoadIcon(0, IDI_APPLICATION);
	wndClassEx.hCursor = LoadCursor(NULL, IDC_ARROW);
	//	wndClassEx.hbrBackground =	(HBRUSH)GetStockObject(WHITE_BRUSH);
	//	wndClassEx.lpszMenuName =	NULL;
	wndClassEx.lpszClassName = className;
	//	wndClassEx.hIconSm =		LoadIcon(0, IDI_APPLICATION);

	RegisterClassEx(&wndClassEx);

	hWnd = CreateWindowEx(NULL,
		className,
		windowName,
		WS_POPUP,//WS_EX_TOPMOST | WS_POPUP,
		200, 200,
		wWidth, wHeight,// set window to new resolution
		NULL,
		NULL,
		hInstance,
		NULL);

	if (!hWnd)
	{
		DWORD err = GetLastError();
		return 0;
	}

	ShowWindow(hWnd, SW_SHOW);

	UpdateWindow(hWnd);

	CoInitializeEx(NULL, COINIT_MULTITHREADED);


	//INIT!!!!!!!!!!!!!!

	pD3D9 = Direct3DCreate9(D3D_SDK_VERSION);
	if (pD3D9 == NULL)
		return 0;

	UINT x32size = pD3D9->GetAdapterModeCount(D3DADAPTER_DEFAULT, D3DFMT_X8R8G8B8);
	D3DDISPLAYMODE dispMode;
	for (UINT i = 0; i < x32size; ++i)
	{
		pD3D9->EnumAdapterModes(D3DADAPTER_DEFAULT, D3DFMT_X8R8G8B8, i, &dispMode);
		if (dispMode.Height == wHeight &&
			dispMode.Width == wWidth &&
			dispMode.RefreshRate == refreshRate)
			break;
	}

	D3DPRESENT_PARAMETERS d3dpp;
	d3dpp.BackBufferWidth = dispMode.Width;
	d3dpp.BackBufferHeight = dispMode.Height;
	d3dpp.BackBufferFormat = D3DFMT_A8R8G8B8;
	d3dpp.BackBufferCount = 1;
	d3dpp.MultiSampleType = D3DMULTISAMPLE_8_SAMPLES; // D3DMULTISAMPLE_NONE;
	d3dpp.MultiSampleQuality = 0;
	d3dpp.SwapEffect = D3DSWAPEFFECT_DISCARD;
	d3dpp.hDeviceWindow = hWnd;
	d3dpp.Windowed = TRUE;
	d3dpp.EnableAutoDepthStencil = TRUE;
	d3dpp.AutoDepthStencilFormat = D3DFMT_D24S8;
	d3dpp.Flags = 0;

	//d3dpp.FullScreen_RefreshRateInHz = D3DPRESENT_RATE_DEFAULT;
	//d3dpp.PresentationInterval = D3DPRESENT_INTERVAL_ONE;

	d3dpp.FullScreen_RefreshRateInHz = D3DPRESENT_RATE_DEFAULT;
	d3dpp.PresentationInterval = D3DPRESENT_INTERVAL_IMMEDIATE;

	HRESULT hr = S_OK;
	if (FAILED(hr = pD3D9->CreateDevice(D3DADAPTER_DEFAULT,
		D3DDEVTYPE_HAL,
		hWnd,
		D3DCREATE_MIXED_VERTEXPROCESSING | D3DCREATE_MULTITHREADED,
		&d3dpp,
		&pDevice)))
	{
		pD3D9->Release();
		return 0;
	}


	IDirect3DVertexBuffer9 *vBuffer;
	int iBufferLen = sizeof(DWORD) * 4 * 10000;
	if (FAILED(hr = pDevice->CreateVertexBuffer(iBufferLen,
		D3DUSAGE_DYNAMIC | D3DUSAGE_WRITEONLY,
		D3DFVF_XYZ | D3DFVF_DIFFUSE,
		D3DPOOL_DEFAULT,
		&vBuffer,
		NULL)))
		return 0;

	//ShowCursor(TRUE);
	//pDevice->ShowCursor(TRUE);
	//ShowCursor(TRUE);
	//SetCursorPos(20, 20);


	//INIT INPUT

	DirectInput8Create(hInstance, DIRECTINPUT_VERSION, IID_IDirectInput8W, (void**)&dInput, NULL);

	dInput->CreateDevice(GUID_SysKeyboard, &dInputKeyboard, NULL);
	dInputKeyboard->SetDataFormat(&c_dfDIKeyboard);
	dInputKeyboard->SetCooperativeLevel(hWnd, /*DISCL_NONEXCLUSIVE | DISCL_BACKGROUND);/*/DISCL_NONEXCLUSIVE | DISCL_FOREGROUND | DISCL_NOWINKEY);


	dInput->CreateDevice(GUID_SysMouse, &dInputMouse, NULL);
	dInputMouse->SetDataFormat(&c_dfDIMouse);
	dInputMouse->SetCooperativeLevel(hWnd, DISCL_NONEXCLUSIVE | DISCL_FOREGROUND); //DISCL_EXCLUSIVE | DISCL_FOREGROUND);
	mouseSpeed = 1.0f;


	//INIT TIME
	currentTime = std::chrono::high_resolution_clock::now();
	lastTime = currentTime;


	// setup gravity
	ColliderAffectorSPtr gravityForce = std::make_shared<StaticAccelerationAffector>("gravity", Acceleration({0.0f, -10.0f}, 0.0f));
	collisionPhysics->registerAffector(gravityForce, AS_ALL);


	std::array<Vector2, 4> points;

	points[0] = { 10.0f, (wHeight / 2 - 10.0f) + 50.0f };
	points[3] = { points[0].x, -points[0].y };
	points[2] = { -points[0].x, -points[0].y };
	points[1] = { -points[0].x, points[0].y };

	/*

	ColliderShapePtr shape = std::make_unique<ConvexPolygon>(&points[0], 4);
	shape->setMass((float)INFINITY);

	ColliderSPtr c = std::make_shared<Collider>();
	c->addShape(std::move(shape));
	c->setPosition(Vector2((wWidth / 2) + 50.0f, 0.0f));
	c->updatePhysicsParams();
	c->color = 0xFFFF0000;

	colliders.emplace_back(c->getName());
	collisionPhysics->addColider(c);

	//===========================================================================

	shape = std::make_unique<ConvexPolygon>(&points[0], 4);
	shape->setMass((float)INFINITY);

	c = std::make_shared<Collider>();
	c->addShape(std::move(shape));
	c->setPosition(Vector2(-((wWidth / 2) + 50.0f), 0.0f));
	c->updatePhysicsParams();
	c->color = 0xFFFF0000;

	colliders.emplace_back(c->getName());
	collisionPhysics->addColider(c);

	//===========================================================================

	points[0] = { (wWidth / 2 - 10.0f) + 50.0f, 10.0f };
	points[3] = { points[0].x, -points[0].y };
	points[2] = { -points[0].x, -points[0].y };
	points[1] = { -points[0].x, points[0].y };

	shape = std::make_unique<ConvexPolygon>(&points[0], 4);
	shape->setMass((float)INFINITY);

	c = std::make_shared<Collider>();
	c->addShape(std::move(shape));
	c->setPosition(Vector2(0.0f, (wHeight / 2) + 50.0f));
	c->updatePhysicsParams();
	c->color = 0xFFFF0000;

	colliders.emplace_back(c->getName());
	collisionPhysics->addColider(c);

	//===========================================================================

	shape = std::make_unique<ConvexPolygon>(&points[0], 4);
	shape->setMass((float)INFINITY);

	c = std::make_shared<Collider>();
	c->addShape(std::move(shape));
	c->setPosition(Vector2(0.0f, -((wHeight / 2) + 50.0f)));
	c->updatePhysicsParams();
	c->color = 0xFFFF0000;

	colliders.emplace_back(c->getName());
	collisionPhysics->addColider(c);

	//===========================================================================


	const char pendulumPointName[COLLIDER_ID_LEN] = "pendulum hook";
	Vector2 pendulumPoint(0.0f);
	ColliderSPtr pendulumHook = std::make_shared<Collider>();
	pendulumHook->setName(pendulumPointName);
	shape = std::make_unique<PointMass>(Vector2(0.0f, 0.0f));
	shape->setMass((float)INFINITY);
	pendulumHook->addShape(std::move(shape));
	pendulumHook->updatePhysicsParams();
	pendulumHook->setPosition(pendulumPoint);
	pendulumHook->color = 0xFFFFFF00;

	float ball1Distance = 100.0f;
	const char pendulumBall1Name[COLLIDER_ID_LEN] = "pendulum ball 1";
	ColliderSPtr pendulumBall1 = std::make_shared<Collider>();
	pendulumBall1->setName(pendulumBall1Name);
	shape = std::make_unique<Circle>(Vector2(0.0f, 0.0f), 25.0f);
	shape->setMass(10.0f);
	pendulumBall1->addShape(std::move(shape));
	pendulumBall1->updatePhysicsParams();
	pendulumBall1->setPosition(Vector2(pendulumPoint.x + ball1Distance, pendulumPoint.y));
	pendulumBall1->color = 0xFFCDCDCD;

	float ball2Distance = 50.0f;
	const char pendulumBall2Name[COLLIDER_ID_LEN] = "pendulum ball 2";
	ColliderSPtr pendulumBall2 = std::make_shared<Collider>();
	pendulumBall2->setName(pendulumBall2Name);
	shape = std::make_unique<Circle>(Vector2(0.0f, 0.0f), 10.0f);
	shape->setMass(6.0f);
	pendulumBall2->addShape(std::move(shape));
	pendulumBall2->updatePhysicsParams();
	pendulumBall2->setPosition(Vector2(pendulumPoint.x + ball1Distance + ball2Distance, pendulumPoint.y));
	pendulumBall2->color = 0xFF00CDCD;



	collisionPhysics->addColider(pendulumHook);
	collisionPhysics->addColider(pendulumBall1);
	collisionPhysics->addColider(pendulumBall2);

	const char pendulumConstraint1Name[CONSTRAINT_ID_LEN] = "pendulum constraint 1";
	ConstraintSPtr pendulumConstraint1 = std::make_shared<DistanceConstraint>(pendulumConstraint1Name, Vector2(0.0f, 0.0f), Vector2(0.0f, 0.0f), ball1Distance);

	collisionPhysics->addConstraint(pendulumConstraint1, pendulumPointName, pendulumBall1Name);


	const char pendulumConstraint2Name[CONSTRAINT_ID_LEN] = "pendulum constraint 2";
	ConstraintSPtr pendulumConstraint2 = std::make_shared<DistanceConstraint>(pendulumConstraint2Name, Vector2(0.0f, 0.0f), Vector2(0.0f, 0.0f), ball2Distance);

	collisionPhysics->addConstraint(pendulumConstraint2, pendulumBall1Name, pendulumBall2Name);


	//===========================================================================
	*/

	const char ballName[COLLIDER_ID_LEN] = "ball";
	ColliderSPtr ball = std::make_shared<Collider>();
	ball->setName(ballName);
	ColliderShapePtr shape = std::make_unique<Circle>(Vector2(0.0f, 0.0f), 0.5f);
	shape->setMass(6.0f);
	ball->addShape(std::move(shape));
	ball->updatePhysicsParams();
	ball->setPosition(Vector2(-30.0f, 10.5f));
	ball->setVelocity(Vector2(0.0f, 0.0f));
	ball->color = 0xFF00CDCD;

	collisionPhysics->addColider(ball);


	const char floorName[COLLIDER_ID_LEN] = "floor";
	ColliderSPtr floor = std::make_shared<Collider>();

	points[0] = { 50.0f, 0.0f };
	points[1] = { -50.0f, 0.0f };
	points[2] = { -50.0f, -2.0f };
	points[3] = { 50.0f, -2.0f };
	shape = std::make_unique<ConvexPolygon>(&points[0], 4);
	shape->setMass((float)INFINITY);

	floor->addShape(std::move(shape));
	floor->setPosition(Vector2(0.0f, -10.0f));
	floor->updatePhysicsParams();
	floor->color = 0xFFFF0000;

	collisionPhysics->addColider(floor);



	//===========================================================================


	MSG msg;
	ZeroMemory(&msg, sizeof(MSG));

	while (msg.message != WM_QUIT)
	{

		if (PeekMessage(&msg, 0, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else
		{
			if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastTime)).count() < (1000 / 60))
				continue;

			//UPDATE INPUT
			HRESULT hr = S_OK;
			hr = dInputKeyboard->Acquire();
			memcpy(keyStateOld, keyState, sizeof(keyStateOld));
			hr = dInputKeyboard->GetDeviceState(256, (void*)keyState);
			//hr = dInputMouse->Acquire();
			//hr = dInputMouse->GetDeviceState(sizeof(DIMOUSESTATE), (LPVOID)&mouseState);
			hr = S_OK;


			/*
			std::wstring text(1000, '\0');
			swprintf_s(text.data(), 1000, L"mouse X: %d, Y: %d, Z: %d, Btn1: %d, Btn2: %d, Btn3: %d, Btn4: %d\r\n", mouseState.lX, mouseState.lY, mouseState.lZ,
				mouseState.rgbButtons[0], mouseState.rgbButtons[1], mouseState.rgbButtons[2], mouseState.rgbButtons[3]);
			OutputDebugString(text.c_str());
			//*/

			// RENDER HERE!!!!!!!!!!!!!

			if (FAILED(hr = pDevice->Clear(0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0x00000000, 1.0f, 0)))
				return 0;
			if (FAILED(hr = pDevice->BeginScene()))
				return 0;


			//D3DMATRIX mView = { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
			D3DMATRIX mView = { 1.0f * viewScale, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f * viewScale, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f * viewScale, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
			D3DMATRIX mProj = { 1.0f / wWidth, 0.0f, 0.0f, 0.0f, 0.0f, 1.f / wHeight, 0.0f, 0.0f, 0.0f, 0.0f, 2.f / 2.f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };

			if (FAILED(hr = pDevice->SetTransform(D3DTS_VIEW, &mView)))
				return 0;
			if (FAILED(hr = pDevice->SetTransform(D3DTS_PROJECTION, &mProj)))
				return 0;


			// render broadphase
			//collisionPhysicsBroadPhase->renderTree(pDevice, vBuffer, iBufferLen);
			// render colision objects
			//for (auto it = colliders.begin(), itEnd = colliders.end(); it != itEnd; ++it)
				//(*it).first->render(pDevice, vBuffer, iBufferLen);

			// render all

			DWORD lighting;
			//DWORD culling;
			pDevice->GetRenderState(D3DRS_LIGHTING, &lighting);
			pDevice->SetRenderState(D3DRS_LIGHTING, FALSE);

			//pDevice->GetRenderState(D3DRS_CULLMODE, &culling);
			//pDevice->SetRenderState(D3DRS_CULLMODE, D3DCULL_CW);



			collisionPhysics->render(pDevice, vBuffer, iBufferLen);



			if (FAILED(hr = pDevice->EndScene()))
				return 0;

			if (FAILED(hr = pDevice->Present(0, 0, 0, 0)))
				return 0;


			lastTime = currentTime;
			currentTime = std::chrono::high_resolution_clock::now();

			static float timer(0.0f);
			static unsigned int fps(0);

			float timestep = std::chrono::duration<float>(currentTime - lastTime).count();
			//float timestep = 1.0f / 30.0f;

			timer += timestep;
			++fps;
			if (timer >= 1.0f)
			{
				std::wstring text(100, '\0');
				swprintf_s(text.data(), 100, L"FPS: %f\r\n", fps/timer);
				OutputDebugString(text.c_str());

				timer = 0.0f;
				fps = 0;
			}

			//OutputDebugString(L"----------NEXT FRAME----------\r\n");

			collisionPhysics->progressTime(timestep);
			//collisionPhysics->progressTime2(timestep);

		}
	}


	//SHUTDOWN

	dInputKeyboard->Unacquire();
	dInputMouse->Unacquire();
	dInput->Release();

	pDevice->Release();
	pD3D9->Release();



	CoUninitialize();

	UnregisterClass(className, hInstance);

	return 0;

};

