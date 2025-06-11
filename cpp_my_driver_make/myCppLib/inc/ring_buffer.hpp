/*
 * ring_buffer.hpp
 *
 *  Created on: Jun 2, 2025
 *      Author: Dogukan AKCA
 */

#ifndef INC_RING_BUFFER_HPP_
#define INC_RING_BUFFER_HPP_

#pragma once
#include <cstdint>
#include <cstddef>

template<typename T, int Size>
class RingBuffer {
private:
    T buffer[Size];
    volatile int head = 0;
    volatile int tail = 0;

public:
    bool put(T data) {
        int next = (head + 1) % Size;
        if (next == tail) return false; // buffer full
        buffer[head] = data;
        head = next;
        return true;
    }

    bool get(T &data) {
        if (head == tail) return false; // buffer empty
        data = buffer[tail];
        tail = (tail + 1) % Size;
        return true;
    }

    bool is_empty() const {
        return head == tail;
    }

    bool is_full() const {
        return ((head + 1) % Size) == tail;
    }
};


#endif /* INC_RING_BUFFER_HPP_ */
