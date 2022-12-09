#include "PhysicsStructs.h"


MaterialPair::MaterialPair(MaterialID _id1, MaterialID _id2) : id1(_id1), id2(_id2)
{};


bool operator==(const MaterialPair& lhs, const MaterialPair& rhs)
{
	return (lhs.id1 == rhs.id1 && lhs.id2 == rhs.id2) || (lhs.id1 == rhs.id2 && lhs.id2 == rhs.id1);
};


bool operator==(const std::pair<Collider*, Collider*> &lhs, const std::pair<Collider*, Collider*> &rhs)
{
	return (lhs.first == rhs.first && lhs.second == rhs.second) || (lhs.first == rhs.second && lhs.second == rhs.first);
};


Features::Features(const Features& rhs) :
	f1(rhs.f1),
	f2(rhs.f2)
{};


Features::Features(const Feature& _f1, const Feature& _f2) :
	f1(_f1),
	f2(_f2)
{};


unsigned int Features::hash() const
{
	return FEATURE_HASH(f1.type, f2.type);
};


DirectionVector::DirectionVector(Vector2 vector) :
	magnitude(vector.length()),
	direction(vector * (1.0f / magnitude))
{};


DirectionVector::DirectionVector(Vector2 _direction, float _magnitude) :
	direction(_direction),
	magnitude(_magnitude)
{};


DirectionVector DirectionVector::operator-() const
{
	return DirectionVector(-direction, magnitude);
};


DirectionVector::operator Vector2 () const
{
	return direction * magnitude;
};


