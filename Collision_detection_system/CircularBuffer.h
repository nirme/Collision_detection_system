#pragma once

#include <cassert>
#include <array>

template <class T, size_t length>
class CircularBuffer
{
protected:
	std::array<T, length> container;
	unsigned int currentIndex;
	unsigned int savedEntries;

	T& current()
	{
		return container[currentIndex];
	};

public:
	CircularBuffer() : 
		currentIndex(0),
		savedEntries(0)
	{};

	void push(T& value)
	{
		current() = value;
		currentIndex = ++currentIndex % length;
		if (++savedEntries > length)
			savedEntries = length;
	};

	T pop()
	{
		assert(savedEntries > 0 && "rollback not possible, buffer empty");
		currentIndex = (length + currentIndex - 1) % length;
		--savedEntries;
		return current();
	};

	const T& at(size_t idx) const
	{
		assert(idx < savedEntries && "index out of bounds");
		return container[(length + currentIndex - idx) % length];
	};

	const T& operator[](size_t idx) const
	{
		assert(idx < savedEntries && "index out of bounds");
		return container[(length + currentIndex - idx) % length];
	};

	void clear()
	{
		currentIndex = 0;
		savedEntries = 0;
	};

};