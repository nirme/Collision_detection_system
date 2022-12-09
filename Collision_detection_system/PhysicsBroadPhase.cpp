#include "PhysicsBroadPhase.h"



PhysicsBroadPhase::Node::Node() :
	box(),
	parent(INVALID_INDEX),
	child{ INVALID_INDEX, INVALID_INDEX },
	boxOwner(nullptr)
{};


float PhysicsBroadPhase::Node::evalArea() const
{
	return box.area();
};


float PhysicsBroadPhase::Node::evalChangeInArea(const AABB &_box) const
{
	return box.merge(_box).area() - box.area();
};



int32_t PhysicsBroadPhase::getSibling(int32_t _nodeId) const
{
	int32_t p = vNodeList[_nodeId].parent;

	// probably dont need this check here?
	if (p != INVALID_INDEX)
		return vNodeList[p].child[0] == _nodeId ? vNodeList[p].child[1] : vNodeList[p].child[0];
	return INVALID_INDEX;
};


void PhysicsBroadPhase::swapChild(int32_t _oldNodeId, int32_t _newNodeId)
{
	int32_t p = vNodeList[_oldNodeId].parent;

	vNodeList[_newNodeId].parent = p;
	vNodeList[_oldNodeId].parent = INVALID_INDEX;

	if (vNodeList[p].child[0] == _oldNodeId)
		vNodeList[p].child[0] = _newNodeId;
	else
		vNodeList[p].child[1] = _newNodeId;
};


void PhysicsBroadPhase::setChildren(int32_t _parentId, int32_t _c0, int32_t _c1)
{
	vNodeList[_c0].parent = _parentId;
	vNodeList[_c1].parent = _parentId;

	vNodeList[_parentId].child[0] = _c0;
	vNodeList[_parentId].child[1] = _c1;
};


void PhysicsBroadPhase::propagateChange(int32_t _nodeId)
{
	int32_t n = vNodeList[_nodeId].isLeaf() ? vNodeList[_nodeId].parent : _nodeId;
	while (n >= 0)
	{
		vNodeList[n].box = vNodeList[vNodeList[n].child[0]].box.merge(vNodeList[vNodeList[n].child[1]].box);
		n = vNodeList[n].parent;
	}
};


void PhysicsBroadPhase::updateBox(int32_t _node, float _boxMargin)
{
	if (!vNodeList[_node].isLeaf())
		vNodeList[_node].box = vNodeList[vNodeList[_node].child[0]].box.merge(vNodeList[vNodeList[_node].child[1]].box);
	else
	{
		vNodeList[_node].box = vNodeList[_node].boxOwner->getAABB();
		vNodeList[_node].box = vNodeList[_node].box.expandAbs(_boxMargin);
	}
};


int32_t PhysicsBroadPhase::getFreeNode()
{
	if (vFreeNodeList.size())
	{
		int32_t out = vFreeNodeList.back();
		vFreeNodeList.pop_back();
		return out;
	}

	int32_t size = (int32_t)vNodeList.size();
	vNodeList.resize(size + 1);
	return size;
};



void PhysicsBroadPhase::freeNode(int32_t _n)
{
	vNodeList[_n].parent = INVALID_INDEX;
	vNodeList[_n].child[0] = INVALID_INDEX;
	vNodeList[_n].child[1] = INVALID_INDEX;
	vNodeList[_n].boxOwner = nullptr;
	vFreeNodeList.emplace_back(_n);
};


void PhysicsBroadPhase::insertNode(int32_t _node)
{
	if (root < 0)
	{
		root = _node;
		return;
	}

	int32_t parent = root;

	// find the leaf node
	while (!vNodeList[parent].isLeaf())
		parent = vNodeList[vNodeList[parent].child[0]].evalChangeInArea(vNodeList[_node].box) < vNodeList[vNodeList[parent].child[1]].evalChangeInArea(vNodeList[_node].box) ?
		vNodeList[parent].child[0] :
		vNodeList[parent].child[1];

	// split leaf node
	int32_t newParent = getFreeNode();
	// update parent with new node
	if (vNodeList[parent].parent >= 0)
		swapChild(parent, newParent);
	else
	{
		vNodeList[newParent].parent = INVALID_INDEX;
		root = newParent;
	}
	// set children on new node
	setChildren(newParent, parent, _node);

	propagateChange(_node);
};


void PhysicsBroadPhase::disconnectLeaf(int32_t _node)
{
	if (vNodeList[_node].parent >= 0)
	{
		int32_t sibling = getSibling(_node);
		int32_t parent = vNodeList[_node].parent;

		// swap node with two children for the sibling
		if (vNodeList[parent].parent >= 0)
		{
			swapChild(parent, sibling);
			// update parent
			propagateChange(vNodeList[sibling].parent);
		}
		else // parent is root so no grantparent available
		{
			root = sibling;
			vNodeList[sibling].parent = INVALID_INDEX;
		}

		freeNode(parent);
	}
	else // node is root so no parent
	{
		root = INVALID_INDEX;
	}
};


void PhysicsBroadPhase::setMargin(float _boxMargin)
{
	boxMargin = _boxMargin;
};


void PhysicsBroadPhase::addCollider(Collider &_object)
{
	int32_t node = getFreeNode();
	_object.setPhysicsBroadPhaseId(node);

	vNodeList[node].boxOwner = &_object;
	vNodeList[node].box = _object.getAABB().expandAbs(boxMargin);

	insertNode(node);
};


void PhysicsBroadPhase::removeCollider(Collider &_object)
{
	int32_t node = _object.getPhysicsBroadPhaseId();

	disconnectLeaf(node);
	freeNode(node);
};


void PhysicsBroadPhase::invalidateColliderNode(int32_t _id)
{
	vInvalidNodeList.emplace_back(_id);
};


void PhysicsBroadPhase::update()
{
	if (unsigned int len = (unsigned int)vInvalidNodeList.size())
	{
		for (unsigned int i = 0; i < len; ++i)
		{
			int32_t n = vInvalidNodeList[i];
			assert((n >= 0 && n < vNodeList.size()) || "incorrect node id in invalidated list");

			AABB newBox = vNodeList[n].boxOwner->getAABB();
			if (!vNodeList[n].box.isContaining(newBox))
			{
				// move sibling up a level
				disconnectLeaf(n);

				// reinsert
				vNodeList[n].box = newBox.expandAbs(boxMargin);
				insertNode(n);
				propagateChange(n);
			}
		}
		vInvalidNodeList.clear();
	}
};


void PhysicsBroadPhase::updateAll()
{
	// invalidate all leaf nodes
	vInvalidNodeList.clear();
	for (int i = 0, iMax = (int)vNodeList.size(); i < iMax; ++i)
	{
		// unused nodes are also not leafs
		if (vNodeList[i].isLeaf())
			vInvalidNodeList.emplace_back(i);
	}

	// update all of them
	update();
};


ColliderList PhysicsBroadPhase::query(const Vector2 &_point) const
{
	if (root < 0) return {};

	ColliderList list;
	std::vector<int32_t> stack;
	stack.reserve(vNodeList.size() >> 1);

	stack.emplace_back(root);
	while (stack.size())
	{
		int32_t node = stack.back();
		stack.pop_back();

		if (vNodeList[node].box.isContaining(_point))
		{
			if (vNodeList[node].isLeaf())
				list.emplace_back(vNodeList[node].boxOwner);
			else
			{
				stack.emplace_back(vNodeList[node].child[0]);
				stack.emplace_back(vNodeList[node].child[1]);
			}
		}
	}

	return list;
};


ColliderList PhysicsBroadPhase::query(const Vector2 &_sectionP1, const Vector2 &_sectionP2) const
{
	if (root < 0) return {};

	ColliderList list;
	std::vector<int32_t> stack;
	stack.reserve(vNodeList.size() >> 1);

	stack.emplace_back(root);
	while (stack.size())
	{
		int32_t node = stack.back();
		stack.pop_back();

		if (vNodeList[node].box.isOverlapping(_sectionP1, _sectionP2))
		{
			if (vNodeList[node].isLeaf())
				list.emplace_back(vNodeList[node].boxOwner);
			else
			{
				stack.emplace_back(vNodeList[node].child[0]);
				stack.emplace_back(vNodeList[node].child[1]);
			}
		}
	}

	return list;
};


ColliderList PhysicsBroadPhase::query(const AABB &aabb) const
{
	if (root < 0) return {};

	ColliderList ret;
	std::vector<int32_t> testIds(10);
	testIds.resize(1);
	testIds[0] = root;

	while (testIds.size())
	{
		int32_t id = testIds.back();
		testIds.pop_back();

		if (vNodeList[id].box.isOverlapping(aabb))
		{
			if (!vNodeList[id].isLeaf())
			{
				testIds.emplace_back(vNodeList[id].child[0]);
				testIds.emplace_back(vNodeList[id].child[1]);
			}
			else
				ret.emplace_back(vNodeList[id].boxOwner);
		}
	}
	return ret;
};


const CollisionList& PhysicsBroadPhase::queryCollisions() const
{
	collisionList.clear();

	if (root < 0 || vNodeList[root].isLeaf())
		return collisionList;

	collisionList.clear();
	collisionList.reserve(previousCollisionCount);

	std::vector<std::pair<int32_t, int32_t>> stack;
	stack.reserve(vNodeList.size());

	// 2 loops to keep repeats from the list
	std::vector<int32_t> stack1;
	stack1.emplace_back(root);
	while (stack1.size())
	{
		auto node = stack1.back();
		stack1.pop_back();

		if (!vNodeList[node].isLeaf())
		{
			stack.emplace_back(vNodeList[node].child[0], vNodeList[node].child[1]);
			stack1.emplace_back(vNodeList[node].child[0]);
			stack1.emplace_back(vNodeList[node].child[1]);
		}
	}

	while (stack.size())
	{
		auto nodes = stack.back();
		stack.pop_back();

		if (vNodeList[nodes.first].box.isOverlapping(vNodeList[nodes.second].box))
		{
			if (vNodeList[nodes.first].isLeaf()) // 1-X
			{
				if (vNodeList[nodes.second].isLeaf()) // 1-1
				{
					if (vNodeList[nodes.first].boxOwner->getAABB().isOverlapping(vNodeList[nodes.second].boxOwner->getAABB()))
					{
						collisionList.emplace_back(vNodeList[nodes.first].boxOwner, vNodeList[nodes.second].boxOwner);
					}
				}
				else // 1-0
				{
					stack.emplace_back(nodes.first, vNodeList[nodes.second].child[0]);
					stack.emplace_back(nodes.first, vNodeList[nodes.second].child[1]);
				}
			}
			else if (vNodeList[nodes.second].isLeaf()) // 0-1
			{
				stack.emplace_back(vNodeList[nodes.first].child[0], nodes.second);
				stack.emplace_back(vNodeList[nodes.first].child[1], nodes.second);
			}
			else // 0-0
			{
				stack.emplace_back(vNodeList[nodes.first].child[0], vNodeList[nodes.second].child[0]);
				stack.emplace_back(vNodeList[nodes.first].child[0], vNodeList[nodes.second].child[1]);
				stack.emplace_back(vNodeList[nodes.first].child[1], vNodeList[nodes.second].child[0]);
				stack.emplace_back(vNodeList[nodes.first].child[1], vNodeList[nodes.second].child[1]);
			}
		}
	}


	previousCollisionCount = (int)(previousCollisionCount + collisionList.size() + 2) >> 1;
	return collisionList;
};


ColliderList PhysicsBroadPhase::queryCollisions(const Collider& c) const
{
	ColliderList coliders;

	int32_t cId = c.getPhysicsBroadPhaseId();
	std::vector<int32_t> stack;
	stack.reserve(vNodeList.size());
	stack.push_back(root);

	const AABB& box = c.getAABB();

	while (stack.size())
	{
		int32_t id = stack.back();
		stack.pop_back();

		if (isOverlapping(box, vNodeList[id].box))
		{
			if (!vNodeList[id].isLeaf())
			{
				stack.push_back(vNodeList[id].child[0]);
				stack.push_back(vNodeList[id].child[1]);
			}
			else if ( id != cId && isOverlapping(box, vNodeList[id].boxOwner->getAABB()))
				coliders.push_back(vNodeList[id].boxOwner);
		}
	}

	return coliders;
};


#if(DIRECT3D_VERSION == 0x0900)

void PhysicsBroadPhase::renderTree(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght)
{

	std::list<int32_t> renderQueue;
	if (root >= 0)
		renderQueue.push_front(root);

	struct VertStruct {
		float x, y, z;
		DWORD color;
	} *vertStruct;


	DWORD lighting;
	_pDevice->GetRenderState(D3DRS_LIGHTING, &lighting);
	_pDevice->SetRenderState(D3DRS_LIGHTING, FALSE);

	_vBuffer->Lock(0, 0, (void**)&vertStruct, D3DLOCK_DISCARD);

	unsigned int primitives = 0;
	static DWORD colorBase = (0xFFFF0000 & (std::rand() << 16)) | (0x0000FFFF & std::rand());
	DWORD color;
	bool boxFlag = false;
	AABB box;

	while (renderQueue.size())
	{
		int32_t n = renderQueue.back();
		renderQueue.pop_back();

		if (!vNodeList[n].isLeaf())
		{
			boxFlag = false;
			renderQueue.push_front(vNodeList[n].child[0]);
			renderQueue.push_front(vNodeList[n].child[1]);
		}

		if ((primitives + 4) * 2 * sizeof(VertStruct) > _bufferLenght)
		{
			_vBuffer->Unlock();

			_pDevice->SetStreamSource(0, _vBuffer, 0, sizeof(VertStruct));
			_pDevice->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE);
			_pDevice->DrawPrimitive(D3DPT_LINELIST, 0, primitives);

			_vBuffer->Lock(0, 0, (void**)&vertStruct, D3DLOCK_DISCARD);

			primitives = 0;
		}

		//if (!boxFlag) // dont render white
		//if (boxFlag) // only render white
		{
			color = boxFlag ? 0xFF8F8F8F : 0xFF000000 | (_rotl(colorBase, (n % 32)));


			box = boxFlag ? vNodeList[n].boxOwner->getAABB() : vNodeList[n].box;

			*vertStruct++ = { box.vMin.x, box.vMin.y, 0.0f, color };
			*vertStruct++ = { box.vMax.x, box.vMin.y, 0.0f, color };

			*vertStruct++ = { box.vMax.x, box.vMin.y, 0.0f, color };
			*vertStruct++ = { box.vMax.x, box.vMax.y, 0.0f, color };

			*vertStruct++ = { box.vMax.x, box.vMax.y, 0.0f, color };
			*vertStruct++ = { box.vMin.x, box.vMax.y, 0.0f, color };

			*vertStruct++ = { box.vMin.x, box.vMax.y, 0.0f, color };
			*vertStruct++ = { box.vMin.x, box.vMin.y, 0.0f, color };

			primitives += 4;
		}


		if (vNodeList[n].isLeaf())
		{
			if (!boxFlag)
			{
				boxFlag = true;
				renderQueue.emplace_back(n);
			}
			else
				boxFlag = false;
		}
	}

	_vBuffer->Unlock();
	_pDevice->SetStreamSource(0, _vBuffer, 0, sizeof(VertStruct));
	_pDevice->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE);
	_pDevice->DrawPrimitive(D3DPT_LINELIST, 0, primitives);


	_pDevice->SetRenderState(D3DRS_LIGHTING, lighting);

};

#endif
