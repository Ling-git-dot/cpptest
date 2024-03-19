
#ifndef MEM_POOP_H
#define MEM_POOP_H

#include <queue>
#include <pthread.h>

namespace zbl{

template<class T, size_t MEM_SIZE>
class MemPool
{
    public:
        MemPool()
        {
            m_lock = PTHREAD_MUTEX_INITIALIZER;//init
            
            expandFreeList();
        }

        ~MemPool()
        {
            pthread_mutex_destroy(&m_lock);
            
            while(!m_free_list.empty())
            {
                if(MEM_SIZE > 1)
                {
                    delete [] (m_free_list.front());
                }
                else
                {
                    delete (m_free_list.front());
                }

                m_free_list.pop();
            }
        }

    public:
        T* alloc()
        {
            pthread_mutex_lock(&m_lock);

            if(m_free_list.empty())
            {
                expandFreeListIn();

            }

            T* t_mem = m_free_list.front();
            m_free_list.pop();

            pthread_mutex_unlock(&m_lock);

            return t_mem;
        }

        void free(T* mem)
        {

            pthread_mutex_lock(&m_lock);

            m_free_list.push(mem);
            pthread_mutex_unlock(&m_lock);
        }

    private:
        void expandFreeList(int num = 20)
        {
            pthread_mutex_lock(&m_lock);


            for(int i=0;i<num;i++)
            {
                if(MEM_SIZE > 1)
                {
                    m_free_list.push(new T [MEM_SIZE]);

                }
                else
                {
                    m_free_list.push(new T);
                }
            }
            pthread_mutex_unlock(&m_lock);

        }

        void expandFreeListIn(int num = 20)
        {

             for(int i=0;i<num;i++)
             {
                 if(MEM_SIZE > 1)
                 {
                     m_free_list.push(new T [MEM_SIZE]);

                 }
                 else
                 {
                     m_free_list.push(new T);
                 }
             }


        }


    private:
        std::queue<T* >     m_free_list;

        pthread_mutex_t     m_lock;
};
}
#endif

