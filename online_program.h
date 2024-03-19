
#ifdef __cplusplus
extern "C"
{
#endif

#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <dirent.h>
#include <unistd.h>
#include <arpa/inet.h>

#pragma pack(1)
struct log
{
    unsigned char flag_online;          //在轨编程标志位
    unsigned char flag_update;          //已完成更新标志位
    unsigned short zg_rightcount; //在轨数据文件正确帧计数
    unsigned short  zg_wrongcount; //在轨数据文件错误帧计数
    unsigned short  bitmap_zg_rightcount; //在轨数据bitmap相对应的正确计数
    unsigned char memory_flag; //缓存标识
    char lib_name[200];        //动态库文件名
    unsigned char version_imageprocess; //图像处理软件版本号

};
#pragma pack()

/**
 *（已完成）获取并保存在轨编程小包
 */
#pragma pack(1)
struct pack
{
    unsigned int fm_head;              //帧头
    unsigned int num_total_pack;       //总包数
    unsigned int index_total_pack;     //总包序号
    unsigned char id_part_pack;        //分包id
    unsigned int index_part_pack;      //分包序号
    unsigned char type_part_pack;      //分包类型
    unsigned short int len_part_pack;  //分包有效数据长度
    unsigned char part_pack_data[460]; //分包的有效数据
    unsigned char checksum;
};
#pragma pack()

#define PACK_FRAME_HEAD 0x1DFCCF1A

//#define LOG_ROUTE "/usr/bin/log_dir/log"
//#define BITMAP_ROUTE "/usr/bin/bitmap_dir/bitmap"
//#define FIRST_PACK_ROUTE "/usr/bin/first_pack_dir/first_pack"

//#define LOG_DIR "/usr/bin/log_dir/"
//#define BITMAP_DIR "/usr/bin/bitmap_dir/"
//#define FIRST_PACK_DIR "/usr/bin/first_pack_dir/"
//#define PACK_DIR "/usr/bin/pack/"
//#define LIB_SAVE_DIR "/usr/bin/lib_dir/"
//#define LIB_BACKUP_DIR "/usr/bin/lib_backup/"



/*
#define LOG_ROUTE "/home/qnh/Downloads/logzg/log_dir/log"
#define BITMAP_ROUTE "/home/qnh/Downloads/logzg/bitmap_dir/bitmap"
#define FIRST_PACK_ROUTE "/home/qnh/Downloads/logzg/first_pack_dir/first_pack"

#define LOG_DIR "/home/qnh/Downloads/logzg/log_dir/"
#define BITMAP_DIR "/home/qnh/Downloads/logzg/bitmap_dir/"
#define FIRST_PACK_DIR "/home/qnh/Downloads/logzg/first_pack_dir/"
#define PACK_DIR "/home/qnh/Downloads/logzg/pack/"
#define LIB_SAVE_DIR "/home/qnh/Downloads/logzg/lib_dir/"
#define LIB_BACKUP_DIR "/home/qnh/Downloads/logzg/lib_backup/"
*/


/*
#define LOG_ROUTE "/mnt/logzg/log_dir/log"  //存放log文件
#define BITMAP_ROUTE "/mnt/logzg/bitmap_dir/bitmap"  //存放bitmap文件
#define FIRST_PACK_ROUTE "/mnt/logzg/first_pack_dir/first_pack"  //存放合成文件

#define LOG_DIR "/mnt/logzg/log_dir/"  //存放log的文件夹
#define BITMAP_DIR "/mnt/logzg/bitmap_dir/"  //存放bitmap的文件夹
#define FIRST_PACK_DIR "/mnt/logzg/first_pack_dir/"  //存放合成的文件夹
#define PACK_DIR "/mnt/logzg/pack/"  //存放小包文件
#define LIB_SAVE_DIR "/mnt/logzg/lib_dir/"
#define LIB_BACKUP_DIR "/mnt/logzg/lib_backup/"
*/

#define LOG_ROUTE "/spiflash/logzg/log_dir/log"  //存放log文件
#define BITMAP_ROUTE "/spiflash/logzg/bitmap_dir/bitmap"  //存放bitmap文件
#define FIRST_PACK_ROUTE "/spiflash/logzg/first_pack_dir/first_pack"  //存放合成文件

#define LOG_DIR "/spiflash/logzg/log_dir/"  //存放log的文件夹
#define BITMAP_DIR "/spiflash/logzg/bitmap_dir/"  //存放bitmap的文件夹
#define FIRST_PACK_DIR "/spiflash/logzg/first_pack_dir/"  //存放合成的文件夹
#define PACK_DIR "/spiflash/logzg/pack/"  //存放小包文件
#define LIB_SAVE_DIR "/spiflash/logzg/lib_dir/"
#define LIB_BACKUP_DIR "/spiflash/logzg/lib_backup/"


/*
#define LOG_ROUTE "/logzg/log_dir/log"  //存放log文件
#define BITMAP_ROUTE "/logzg/bitmap_dir/bitmap"  //存放bitmap文件
#define FIRST_PACK_ROUTE "/logzg/first_pack_dir/first_pack"  //存放合成文件

#define LOG_DIR "/logzg/log_dir/"  //存放log的文件夹
#define BITMAP_DIR "/logzg/bitmap_dir/"  //存放bitmap的文件夹
#define FIRST_PACK_DIR "/logzg/first_pack_dir/"  //存放合成的文件夹
#define PACK_DIR "/logzg/pack/"  //存放小包文件
#define LIB_SAVE_DIR "/logzg/lib_dir/"
#define LIB_BACKUP_DIR "/logzg/lib_backup/"
*/




extern unsigned char getRadarVersion();
extern unsigned char getImgVersion();

extern int initOnlineProgram(); //在轨编程指令
extern int checkOnline();
extern int processReceivedPack(unsigned char *ptr_pack, int len);
extern int executeOnlineProgram();//在轨编程完整实施指令
extern int setLog(const char route_log[], int n);//准备断电指令
extern int getLog(const char route_log[], int n);//恢复上次的在轨编程状态，随程序启动，不需要指令
extern int exitOnlineProgram();//退出在轨编程指令
extern  int processReceivedPackOut(unsigned char *ptr_pack, int len);
extern int ExecuteCMD(const char *cmd, char *result);
extern int getBitMap_BlockData(const char route_bitmap[], int n, unsigned char* ptr_bitmap_data);
extern int  exectProgrammd5(char *filedata,char *filedatamd5);
extern int  exectProgrammd5test(char *filedata,char *filedatamd5);
#ifdef __cplusplus
}
#endif
