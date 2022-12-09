#pragma once

#include <memory>


template <class T, int ItemsPerPage>
class MemoryPool
{
private:
	std::unique_ptr<void, void(*)(void*)> memory;
	void* memoryPtr;

public:
	MemoryPool() :
		memory(std::malloc(sizeof(T) * ItemsPerPage), [](void* ptr) { std::free(ptr); }),
		memoryPtr = memory.get();
	{};



};
