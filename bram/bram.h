#ifndef BRAM_H
#define BRAM_H

#ifdef __cplusplus
extern "C" {
#endif


#define MAX_MEM_LEN 1*1024


#pragma pack(1)
typedef struct bramInfo{
    unsigned int baseAddr;
    int memsize;
    int mmapfd;
    int max_size;
    void *mapSaddr;
    unsigned char flag;
}bramInfo;


#pragma pack()



int initBram(bramInfo *pBramInfo,unsigned int baseAddr,int memsize);
int writeBram(bramInfo *ptrBramInfo,unsigned char data[],int offset,int len);
int readBram(bramInfo *ptrBramInfo,unsigned char data[],int offset,int len);


#ifdef __cplusplus
}
#endif

#endif // BRAM_H
