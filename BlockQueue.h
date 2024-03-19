
#ifndef BLOCK_QUEUE_H
#define BLOCK_QUEUE_H

#include <iostream>
#include <pthread.h>
#include <queue>

namespace zbl{

template<class T, size_t MAX_SIZE>
class BlockQueue
{
    public:
        BlockQueue()
        {
            m_lock = PTHREAD_MUTEX_INITIALIZER;//init

            m_consumer_cond = PTHREAD_COND_INITIALIZER;
            m_producer_cond = PTHREAD_COND_INITIALIZER;
        }

        ~BlockQueue()
        {
            pthread_mutex_destroy(&m_lock);
            pthread_cond_destroy(&m_consumer_cond);
            pthread_cond_destroy(&m_producer_cond);
        }

    public:
        void push(const T& data)
        {
            pthread_mutex_lock(&m_lock);

            while(m_safe_que.size() >= MAX_SIZE)
            {
                pthread_cond_wait(&m_consumer_cond, &m_lock);
            }

            m_safe_que.push(data);

            pthread_mutex_unlock(&m_lock);
            pthread_cond_signal(&m_producer_cond);
        }

        T pop()
        {
            pthread_mutex_lock(&m_lock);

            while(m_safe_que.empty())
            {
                pthread_cond_wait(&m_producer_cond, &m_lock);
            }

            T t_data = m_safe_que.front();
            m_safe_que.pop();

            pthread_mutex_unlock(&m_lock);
            pthread_cond_signal(&m_consumer_cond);

            return t_data;
        }

    private:
        std::queue<T>           m_safe_que;

        pthread_mutex_t         m_lock;//lock

        pthread_cond_t          m_consumer_cond;
        pthread_cond_t          m_producer_cond;
};
}

#endif

