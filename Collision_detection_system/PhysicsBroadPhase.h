#pragma once

#include "Constants.h"
#include <algorithm>
#include <vector>
#include <list>
#include <utility>
#include "Vector2.h"
#include "AABB.h"
#include "Collider.h"

#include <unordered_set>
#include <d3d9.h>
#include <random>


// add margin for AABB queries, impulse dynamics need additional margin of error for constrained colliders


//typedef std::list<Collider*> ColliderList;
typedef std::vector<Collider*> ColliderList;
//typedef std::list<std::pair<Collider*, Collider*>> CollisionList;
typedef std::vector<std::pair<Collider*, Collider*>> CollisionList;

class PhysicsBroadPhase
{
private:
	struct Node
	{
		// change to box owner
		AABB box;
		int32_t parent;
		int32_t child[2];
		Collider *boxOwner;


		Node();

		inline bool isLeaf() const {
			return boxOwner != nullptr;
		};
		
		float evalArea() const;
		float evalChangeInArea(const AABB &_box) const;
	};


	int32_t root;
	std::vector<Node> vNodeList;
	std::vector<int32_t> vFreeNodeList;
	std::vector<int32_t> vInvalidNodeList;
	float boxMargin;

	mutable CollisionList collisionList;
	mutable unsigned int previousCollisionCount;


	int32_t getSibling(int32_t _nodeId) const;
	void swapChild(int32_t _oldNodeId, int32_t _newNodeId);
	void setChildren(int32_t _parentId, int32_t _c0, int32_t _c1);
	void propagateChange(int32_t _nodeId);
	void updateBox(int32_t _node, float _boxMargin);

	int32_t getFreeNode();

	void freeNode(int32_t _n);
	void insertNode(int32_t _node);
	void disconnectLeaf(int32_t _node);

public:
	inline PhysicsBroadPhase() : root(INVALID_INDEX), previousCollisionCount(1){};

	void setMargin(float _boxMargin);

	void addCollider(Collider &_object);
	void removeCollider(Collider &_object);

	// invalid node list must be already populated
	void invalidateColliderNode(int32_t _id);
	void update();
	void updateAll();


	ColliderList query(const Vector2 &_point) const;
	ColliderList query(const Vector2 &_sectionP1, const Vector2 &_sectionP2) const;
	ColliderList query(const AABB &aabb) const;

	const CollisionList& queryCollisions() const;
	const CollisionList& queryCollisions(float _margin) const;
	ColliderList queryCollisions(const Collider& c) const;

#if(DIRECT3D_VERSION == 0x0900)
	void renderTree(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght);
#endif

};