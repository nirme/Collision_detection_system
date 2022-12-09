#pragma once

#include <cassert>


template <typename T>
class Singleton
{
private:
	Singleton(const Singleton<T>&);
	Singleton& operator=(const Singleton<T>&);

protected:
	static T* impl;

public:
	Singleton()
	{
		assert(!impl && "Singleton instance already exist");
		impl = static_cast<T*>(this);
	}

	~Singleton()
	{
		assert(impl && "Singleton instance doesn't exist");
		impl = 0;
	}

	static T& getSingleton() { return *impl; };
	static T* getSingletonPtr() { return impl; };
};
