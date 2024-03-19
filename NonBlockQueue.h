
#ifndef NON_BLOCK_QUEUE_H
#define NON_BLOCK_QUEUE_H

#include <iostream>
#include <pthread.h>
#include <queue>

namespace zbl{

template<class T, size_t MAX_SIZE>
class NonBlockQueue
{
    public:
        NonBlockQueue()
        {
            m_lock = PTHREAD_MUTEX_INITIALIZER;//init
        }

        ~NonBlockQueue()
        {
            pthread_mutex_destroy(&m_lock);
        }

    public:
        bool full()
        {
            pthread_mutex_lock(&m_lock);

            bool t_full = (m_safe_que.size() >= MAX_SIZE);

            pthread_mutex_unlock(&m_lock);

            return t_full;
        }

        bool empty()
        {
            pthread_mutex_lock(&m_lock);

            bool t_empty =  m_safe_que.empty();

            pthread_mutex_unlock(&m_lock);

            return t_empty;
        }

        void push(const T& data)
        {
            pthread_mutex_lock(&m_lock);

            m_safe_que.push(data);


            pthread_mutex_unlock(&m_lock);
        }

        T pop()
        {
            pthread_mutex_lock(&m_lock);

            T t_data = m_safe_que.front();
            m_safe_que.pop();


            pthread_mutex_unlock(&m_lock);

            return t_data;
        }

    private:
        std::queue<T>           m_safe_que;

        pthread_mutex_t         m_lock;//lock
};
}

#endif

