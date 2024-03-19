#ifndef LOCK_QUEUE_H
#define LOCK_QUEUE_H

#include <iostream>
#include <pthread.h>
#include <queue>

namespace zbl{

template<class T>
class LockQueue
{
    public:
        LockQueue(const size_t& max_size)
        : m_max_size(max_size)
        {
            m_lock = PTHREAD_MUTEX_INITIALIZER;//init

            m_consumer_cond = PTHREAD_COND_INITIALIZER;
            m_producer_cond = PTHREAD_COND_INITIALIZER;
        }

        ~LockQueue()
        {
            pthread_mutex_destroy(&m_lock);
            pthread_cond_destroy(&m_consumer_cond);
            pthread_cond_destroy(&m_producer_cond);
        }

    public:
        //T front()
        //{
        //    pthread_mutex_lock(&m_lock);

        //    while(m_safe_que.empty())
        //    {
        //        pthread_cond_wait(&m_producer_cond, &m_lock);
        //    }

        //    T t_data = m_safe_que.front();

        //    pthread_mutex_unlock(&m_lock);

        //    return t_data;
        //}

        void push(const T& data)
        {
            pthread_mutex_lock(&m_lock);

            while(m_safe_que.size() >= m_max_size)
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
        size_t                  m_max_size;

        pthread_mutex_t         m_lock;//lock

        pthread_cond_t          m_consumer_cond;
        pthread_cond_t          m_producer_cond;
};
}

#endif

