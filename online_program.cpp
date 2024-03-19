#include "online_program.h"

#include "receive_uart.h"
//#include <map>
#include <string.h>
//std::map<std::string, std::string>  t_test_map;

#define TM_BITMAP_SIZE 272 //突发遥测数据大小
#define FLAG_ENTER_ONLINE 0xAA
#define FLAG_EXIT_ONLINE 0x55


unsigned int fd_first_pack_lenth=0; //在轨的文件总长度




//分段标识
unsigned int segmet_bj=0;  //首先要有段第一段
unsigned int segmet_bj_status=0;  //首先要有段第一段
unsigned int  segmet_bj_lenth=0;
unsigned int  segmet_bj_offset=0;

#define CMD_RESULT_BUF_SIZE 2048

int ExecuteCMD(const char *cmd, char *result)
{
    int iRet = -1;
    char buf_ps[CMD_RESULT_BUF_SIZE];
    char ps[CMD_RESULT_BUF_SIZE] = {0};
    FILE *ptr;

    strcpy(ps, cmd);

    if((ptr = popen(ps, "r")) != NULL)
    {
        while(fgets(buf_ps, sizeof(buf_ps), ptr) != NULL)
        {
           strcat(result, buf_ps);
           if(strlen(result) > CMD_RESULT_BUF_SIZE) break;
        }

        iRet = pclose(ptr);
        ptr = NULL;
        if(iRet == 0)
        {
           iRet = 0;  // 处理成功
        }
        else
        {
           #ifdef  IF_PRINTF_MONITOR
           printf("Command execution failed\n");
           #endif
           iRet = -1; // 处理失败
       }

    }
    else
    {
        #ifdef  IF_PRINTF_MONITOR
        printf("popen %s error\n", ps);
        #endif
        iRet = -1; // 处理失败
    }
    return iRet;
}

/**
 * @brief 读取log文件中的全部内容，读取至DDR中
 *
 * @param route_log
 * @param n
 * @param ptr_log 指向DDR中存放log数据的结构
 * @return int 返回-1表示log文件不存在，返回0表示读取成功
 */
int getLog_All(const char route_log[], int n,struct log *ptr_log)
{
    FILE *fd_log = fopen(route_log, "rb"); //仅可读
    if (fd_log == NULL)
    {
        //DDR中的log清0
        memset(ptr_log, 0, sizeof(struct log));
        return -1;
    }
    fseek(fd_log, 0, SEEK_SET);
    fread(ptr_log, sizeof(*ptr_log), 1, fd_log);
    fclose(fd_log);
    return 0;
}

int getLog(const char route_log[],int n)
{
    int ret = getLog_All(route_log,n, &log_data);
    if (ret != 0)
    {
        #ifdef  IF_PRINTF_MONITOR
        printf("getLog : open %s file failed! The file does not exist! \n", route_log);
        #endif
    }
    else
    {
        #ifdef  IF_PRINTF_MONITOR
        printf("getLog: recover the last time online program status. \n");
        #endif
    }
    return ret;
}

int setLog_All(const char route_log[], int n, const struct log *ptr_log)
{
    FILE *fd_log = fopen(route_log, "wb");
    if (fd_log == NULL)
    {
        return -1;
    }
    fseek(fd_log, 0, SEEK_SET);
    fwrite(ptr_log, sizeof(*ptr_log), 1, fd_log);
    fclose(fd_log);
    return 0;
}

/**
 * @brief 将内存中的struct log log_data数据写入log文件中
 *
 * @param route_log
 * @param n
 * @return int
 */
int setLog(const char route_log[], int n)
{
    int ret = setLog_All(route_log, n, &log_data);
    if (ret != 0)
    {
        #ifdef  IF_PRINTF_MONITOR
        printf("setLog_All : open %s file failed! The file does not exist! \n", route_log);
        #endif
    }
    return ret;
}

/**
 * @brief （已完成）检查log文件中在轨编程标志位
 * 标志位为0xAA，则处于在轨编程状态，其余情况为非在轨编程状态
 *
 * @param route_log log文件所在的绝对路径
 * @param n 路径字符串的长度
 * @return int 在轨编程状态返回1，其余状态返回0
 */
int checkLog_FlagOnline(const struct log *ptr_log)
{
    unsigned char flag = ptr_log->flag_online;
    if (flag == FLAG_ENTER_ONLINE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
/**
 * @brief （已完成）检查当前是否在在轨编程状态
 *
 * @return int 处于在轨编程状态返回1，其余状态返回0
 */
int checkOnline()
{
    int ret = checkLog_FlagOnline(&log_data);
    return ret;
}
/**
 * @brief （已完成）设置log文件中的在轨编程标志位
 *
 * @param route_log log文件所在的绝对路径
 * @param n 路径字符串的长度
 * @param flag 需要写入的标志位的值，0xAA进入在轨编程，0x55退出在轨编程
 * @return int
 */
int setLog_FlagOnline(struct log *ptr_log, unsigned char flag)
{
    //设置标志位
    ptr_log->flag_online = flag;
    return 0;
}




/**
 * @brief （已完成）清空指定文件夹
 *
 */
int cleanDir(const char *dir, int n)
{
    DIR *dp;
    struct dirent *entry;
    struct stat statbuf;
    if ((dp = opendir(dir)) == NULL)
    {
        mkdir(dir, 0777);
        if ((dp = opendir(dir)) == NULL)
        {
            #ifdef  IF_PRINTF_MONITOR
            printf("opendir error\n");
            #endif
            return -1;
        }
    }
    if (chdir(dir) != 0)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("chdir error\n");
        #endif
        return -1;
    }
    while ((entry = readdir(dp)) != NULL)
    {
        lstat(entry->d_name, &statbuf);
        if (S_ISREG(statbuf.st_mode))
        {
            remove(entry->d_name);
        }
    }
    return 0;
}

/**
 * @brief （已完成）创建在轨上注所需要的文件夹与文件
 * 包含于initOnlineProgram()中，不单独使用
 *
 */
static int makeOnlineProgramDir()
{
    //创建文件夹
    extern int errno;
    errno = 0;
    int ret = mkdir(LOG_DIR, 0777);
    if(ret<0)
    {
        #ifdef  IF_PRINTF_MONITOR
        printf("%s\n",strerror(errno));
        #endif
    }
    ret |= mkdir(BITMAP_DIR, 0777);
    ret |= mkdir(FIRST_PACK_DIR, 0777);
    ret |= mkdir(LIB_SAVE_DIR, 0777);
   // ret |= mkdir(LIB_BACKUP_DIR, 0775);
    ret |= mkdir(PACK_DIR, 0777);
    if (ret < 0 && errno != EEXIST)
    {
        #ifdef  IF_PRINTF_MONITOR
        printf("make OnlineProgram Dir failed ! \n");
        printf("errno:%d\n",errno);
        #endif
        return -1;
    }
    //清空文件夹
    cleanDir(LOG_DIR, sizeof(LOG_DIR));
    cleanDir(BITMAP_DIR, sizeof(BITMAP_DIR));
    cleanDir(FIRST_PACK_DIR, sizeof(FIRST_PACK_DIR));
    cleanDir(LIB_SAVE_DIR, sizeof(LIB_SAVE_DIR));
    //cleanDir(LIB_BACKUP_DIR, sizeof(LIB_SAVE_DIR));
    cleanDir(PACK_DIR, sizeof(PACK_DIR));

    return 0;
}

/**
 * @brief （已完成）创建在轨编程所需要的文件
 * 包含于initOnlineProgram()中，不单独使用
 * 打开方式为"wb"，清空原文件的内容
 * wb只写
 * rb只读
 * wb+ 为读写打开一个新的二进制文件
 * rb+为读写打开一个二进制文件
 *
 */
static int createOnlineProgramFile()
{
    //创建并清零log文件
    FILE *fd_log = fopen(LOG_ROUTE, "wb+");
    for (unsigned int i = 0; i < sizeof(struct log); i++)
    {
        putc(0, fd_log);
    }
    fclose(fd_log);
    //创建bitmap文件  
    FILE *fd_bitmap = fopen(BITMAP_ROUTE, "wb+");

    fclose(fd_bitmap);
    //创建first_pack文件
    FILE *fd_first_pack = fopen(FIRST_PACK_ROUTE, "wb+");
    fclose(fd_first_pack);
    //返回结果
    if ((fd_log && fd_bitmap && fd_first_pack) == 0)
    {
        perror("createOnlineProgramFile:");
        return -1;
    }
    else
    {
        return 0;
    }
}



/**
 * @brief （已完成）在轨编程初始化
 *
 * @return int 检查标志位，已处于在轨编程状态，标志位未变化返回1;未处于在轨编程状态，标志位变化返回0
 */
int initOnlineProgram()
{
    //检测log文件在轨道编程标志位
    int ret=getLog(LOG_ROUTE, sizeof(LOG_ROUTE));

    //读取文件监测所在状态
    if (checkLog_FlagOnline(&log_data) && ret==0)
    {
        #ifdef  IF_PRINTF_MONITOR
        printf("The monitor has already been in Online Program mode! \n");
        #endif
        return 1;
    }
    else //如果不处于在轨编程状态
    {
        //创建并清理相关文件夹
        makeOnlineProgramDir();
        //创建相关文件
        createOnlineProgramFile();
        getLog_All(LOG_ROUTE, sizeof(LOG_ROUTE), &log_data);
        setLog_FlagOnline(&log_data, FLAG_ENTER_ONLINE);
        zg_excut=0;

        #ifdef  IF_PRINTF_MONITOR
        printf("The monitor ENTER Online Program mode! \n");
        #endif

        //map清空
        /*
        while (!t_test_map.empty())
        {

            t_test_map.erase(t_test_map.begin());
        }
        */

    }
    return 0;
}

/*********************************************
 *
 * 1.在轨编程方案实施
 * ******************************************/
int DoOnlineProgram()
{
    return 0;
}
/**
 * @brief 退出在轨编程，标志位置0x55
 *
 * @return int
 */
int exitOnlineProgram()
{
    //（待修改）（检测步骤可以省略）检测log文件在轨道编程标志位
    int ret = checkLog_FlagOnline(&log_data);
    if (ret == 0)
    {
        #ifdef  IF_PRINTF_MONITOR
        printf("The monitor has already not been in Online Program mode! \n");
        #endif
        return 1;
    }
    //设置标志位
    setLog_FlagOnline(&log_data, FLAG_EXIT_ONLINE);
    //
    #ifdef  IF_PRINTF_MONITOR
    printf("The monitor EXIT Online Program mode! \n");
    #endif

    //并将此状态写入文件

    return 0;
}


/**
 * @brief （已完成）根据size将bitMap文件中的数据置0
 *
 * @param route_bitmap bitMap文件路径
 * @param n 文件路径字符串的长度
 * @param size bitMap覆盖的大小，以bit为单位
 * @return int 清0成功返回0
 */
int clearBitMap(const char route_bitmap[], int n, int size)
{
    //检测有效性
    FILE *fd_bitmap = fopen(route_bitmap, "rb+");
    if (fd_bitmap == NULL)
    {
        #ifdef  IF_PRINTF_MONITOR
        printf("open fd_bitmap %s file failed! \n", route_bitmap);
        #endif
        return -1;
    }
    fseek(fd_bitmap, 0, SEEK_SET);
    int num_char = size / 8 + ((size % 8) == 0 ? 0 : 1); //需要写入的字节数
    #ifdef  IF_PRINTF_MONITOR
    printf("num_char : %d \n", num_char);
    #endif
    for (int i = 0; i < num_char; i++)
    {
        putc(0, fd_bitmap);
    }
    fclose(fd_bitmap);
    return 0;
}

/**
 * @brief （已完成）设置bitMap文件中的某1位
 *
 * @param route_bitmap bitmap文件路径
 * @param n route_bitmap文件路径字符串的长度
 * @param offset 待修改bitMap的索引，从0开始
 * @param value 设置的值
 * @return int 返回负值代表错误，返回0表示该位改变，返回1表示该位未改变
 */
int setBitMap(const char route_bitmap[], int n, int offset, int value)
{
    //检测路径有效性
    FILE *fd_bitmap = fopen(route_bitmap, "rb+");
    if (fd_bitmap == NULL)
    {
        #ifdef  IF_PRINTF_MONITOR
        printf("open fd_bitmap file failed! \n");
        #endif
        return -2;
    }

    int flag_nochange = 0;
    //写入数据，将其分为“整数部分”和“小数部分”
    int integer_bit_offset = offset / 8;
    int decimal_bit_offset = 7 - offset % 8; //按大端序

    unsigned char ch;
    //查看现在文件超度
    fseek(fd_bitmap, 0, SEEK_END);
    unsigned int size_bitmap = ftell(fd_bitmap);

    if(integer_bit_offset>=size_bitmap)
    {
        ch=0; //此文件内还没有此数
        fseek(fd_bitmap, integer_bit_offset, SEEK_SET);
    }
    else
    {
        fseek(fd_bitmap, integer_bit_offset, SEEK_SET);
        //根据实际数据长度发送bitmap
        ch = getc(fd_bitmap);

    }
    #ifdef  IF_PRINTF_MONITOR
    printf("chbefore : 0x%02x \n", ch);
    #endif
    if (value <= 0) //置0
    {
        ch = ch & ~(1 << decimal_bit_offset);
    }
    else //置1
    {
        if ((ch & (1 << decimal_bit_offset)) != 0) //注意运算符优先级
        {
            flag_nochange = 1; //该位置的bit已经为1, 不需要改变
        }
        else
        {
            ch = ch | (1 << decimal_bit_offset);
        }
    }
   // fseek(fd_bitmap, -1L, SEEK_CUR);
    fseek(fd_bitmap, integer_bit_offset, SEEK_SET);
    #ifdef  IF_PRINTF_MONITOR
    printf("ch : 0x%02x \n", ch);
    #endif
    putc(ch, fd_bitmap); //写回bitmap
    fclose(fd_bitmap);
    return flag_nochange;
}


int getBitMap_BlockData(const char route_bitmap[], int n, unsigned char* ptr_bitmap_data)
{
    //检测有效性
    FILE *fd_bitmap = fopen(route_bitmap, "rb+");
    if (fd_bitmap == NULL)
    {
        #ifdef  IF_PRINTF_MONITOR
        printf("open fd_bitmap %s file failed! \n", route_bitmap);
        #endif
        return -1;
    }
    //获取bitmap文件长度
    fseek(fd_bitmap, 0, SEEK_END);
    unsigned int size_bitmap = ftell(fd_bitmap);
    fseek(fd_bitmap, 0, SEEK_SET);
    //获取bitmap数据


    fread(ptr_bitmap_data, sizeof(unsigned char), size_bitmap, fd_bitmap);
    fclose(fd_bitmap);
    return size_bitmap;
}

/**************************************************************************************************
 * 数据格式形式
 * 1.帧头 4字节 0x1acffc1d
 * 2.长度 4字节
 * 3.动态库文件名 200字节
 * 4.版本号  4字节
 * 5.数据区
 * 6.帧尾 0x12345678
 * 7.校验和
 * 多个so文件循环
 *
 *
 *
 * ***********************************************************************************************/
int createLibraryFileOut(const char save_file_path[], int n,unsigned char *data_in,unsigned int lenth)
{


    unsigned int cur=0;
    unsigned int file_lenth;
    unsigned int file_lenth_temp;


    //获取文件长度

    for(;cur+17<=lenth;)  //除了数据区有17字节
    {
         //验证fd_first_pack文件头，校验和，文件结尾
         unsigned int fm_head = 0; //文件头
         memcpy(&fm_head,data_in+cur,4);
         if (fm_head != htonl(0x1ACFFC1D))
         {
             #ifdef  IF_PRINTF_MONITOR
             printf("createLibraryFile: Wrong frame head! \n");
             #endif
             return -1;
         }



         memcpy(&file_lenth_temp, data_in+4+cur,4);
         file_lenth=htonl(file_lenth_temp);

         //判断下是否出现超出文件大小的情况
         if(file_lenth+17+cur>lenth)
         {
             #ifdef  IF_PRINTF_MONITOR
             printf("exceed the size of file! \n");
             #endif
             break;
         }

         //判断校验和
         unsigned char checksum_compute=0;
         unsigned char checksum_cap=data_in[cur+16+200+file_lenth];
         for (unsigned int i = 0; i <(file_lenth+16+200); i++)
         {
             checksum_compute += data_in[cur+i];
         }
         if (checksum_cap != checksum_compute)
         {
             #ifdef  IF_PRINTF_MONITOR
             printf("createLibraryFile: Wrong checksum! \n");
             #endif
             return -1;
         }

         unsigned int fm_tail = 0; //文件结尾

         memcpy(&fm_tail,data_in+cur+12+200+file_lenth,4);

         if (fm_tail != htonl(0x12345678))
         {
             #ifdef  IF_PRINTF_MONITOR
             printf("createLibraryFile: Wrong frame tail! \n");
             #endif
             return -1;
         }

         //获取动态库文件名并更新log，文件名为待放置的全路径，比如libopencv_core.so.3.4.10
         char lib_name[200] = {0};
         memcpy(lib_name,data_in+cur+8,200);

         //获取文件版本号并更新log
         unsigned char file_version[4] = {0};
         memcpy(file_version,data_in+cur+8+200,4);
         log_data.version_imageprocess=file_version[3];
         #ifdef  IF_PRINTF_MONITOR
         printf("file_version[0]:%x file_version[3]:%x\n",file_version[0],file_version[3]);
         #endif


         //保存动态库文件至save_file_path
         char lib_save_route[500] = {0};
         strncpy(lib_save_route, save_file_path, n);  //存储库文件的路径/spiflash
         strcat(lib_save_route, lib_name);  //加上库名字


         FILE *lib_file = fopen(lib_save_route, "wb+");

         fwrite(data_in+cur+12+200,file_lenth,1,lib_file);
         //关闭文件
         fclose(lib_file);  //关闭第一个动态库
         chmod(lib_save_route,S_IRUSR|S_IWUSR|S_IXUSR);
         cur+=(17+200+file_lenth);
    }
    return 0;
}



//最大的在轨文件大小4M 4*1024*1024
#define data_zh_number 8*1024*1024  // 最大多少设置
unsigned char data_zh[data_zh_number]; //用于接收表7的整个文件传输数据域 （其实是一部分空间文件数据段）
#define segement_number_max 18683  //8M数据量，一次449，文件大小479

int processReceivedPackOut(unsigned char *ptr_pack, int len)
{
    //表8或者表9
    unsigned char group_id;
    //static unsigned int status_flag=0;  //用于控制状态机
    //static unsigned int frame_offset=0;
    //static unsigned int segment_lenthall=0;
    unsigned int frame_lenth;
    //unsigned int bitmap_number;
    //static unsigned int count_number;
   // unsigned int count_number_temp;
    unsigned short cap_index;
    unsigned int segment_number; //段序号
    unsigned int segment_offset_address; //偏移地址
    unsigned short segment_lenth=0;  //文件长度
    int flag_nochange=0;





    frame_lenth=((ptr_pack[4]<<8)|ptr_pack[5])+1;  //包长为该数加1
    group_id=((ptr_pack[2]>>6)&0x3); //分组标识
    cap_index=((((ptr_pack[2])&0x3f)<<8)|ptr_pack[3]);
    segment_number=((ptr_pack[10]<<24)|(ptr_pack[11]<<16)|(ptr_pack[12]<<8)|(ptr_pack[13]));  //段序号
    segment_offset_address=((ptr_pack[14]<<24)|(ptr_pack[15]<<16)|(ptr_pack[16]<<8)|(ptr_pack[17]));  //段偏移地址
    segment_lenth=((ptr_pack[18]<<8)|(ptr_pack[19]));  //文件长度

    #ifdef  IF_PRINTF_MONITOR
    printf("segment_offset_address:%d ,segment_lenth:%d\n",segment_offset_address,segment_lenth);
    #endif

    unsigned char sum_all=checkSum(ptr_pack+6,frame_lenth-1,*(ptr_pack+6+frame_lenth-1));


    //判断下2048
    if(sum_all==0xaa  && segment_number<=segement_number_max)
    {
      flag_nochange = setBitMap(BITMAP_ROUTE, sizeof(BITMAP_ROUTE),segment_number, 1); // bitmap相应位置放置1
      log_data.zg_rightcount++;

      if(flag_nochange!=1)
       {
          log_data.bitmap_zg_rightcount++;
      }
    }
    else
    {
        log_data.zg_wrongcount++;
    }
    #ifdef  IF_PRINTF_MONITOR
    printf("log_data.zg_rightcount:%d log_data.zg_wrongcount:%d\n",log_data.zg_rightcount,log_data.zg_wrongcount);
    #endif

    //数据传输失败就不接收
    //data_zh=new unsigned char[len];




        if((segment_offset_address+segment_lenth)<data_zh_number)  //超过8M，偏移量错了
        {

            FILE *fd_first_pack = fopen(FIRST_PACK_ROUTE, "rb+"); //文件IO设置为可读可写模式
            if (fd_first_pack == NULL)
            {
                #ifdef  IF_PRINTF_MONITOR
                printf("processReceivedPack: the %s doesn't exist! \n", FIRST_PACK_ROUTE);
                #endif
                return -1;
            }
            fseek(fd_first_pack, segment_offset_address, SEEK_SET);
            fwrite(ptr_pack+20, segment_lenth, 1, fd_first_pack);
            fflush(fd_first_pack);
            fclose(fd_first_pack);




           // frame_offset+=frame_lenth;

         }
        else
        {
            #ifdef  IF_PRINTF_MONITOR
            printf("包头数据过长 %d %d\n",frame_offset,frame_lenth);
            #endif

        }


       //存小包
       //（可选）保存小包文件
       #if 0

           char pack_name[300] = PACK_DIR;
           char pack_index_char[10] = {0};
           sprintf(pack_index_char, "%d", cap_index);
           strcat(pack_name, pack_index_char);
           printf("pack_name:%s\n\n", pack_name);
           FILE *pack = fopen(pack_name, "wb"); //文件IO设置为写模式
          // fseek(pack, 0L, SEEK_SET);
           fwrite(ptr_pack+20,segment_lenth,1,pack);
           fflush(pack);
           fclose(pack); //关闭文件
      #endif




     return 0;
}


//2023.12.15完成数据的替换以及校验和的写入
#if 0
void  exectProgrammd5(char *filedata,char *filedatamd5)
{

    // 构建shell命令
     char command[512];
     //md5sum /spiflash/image_process>/spiflash/image_process.txt.md5 &&sed -i 's/^\([^ ]* [^ ]*\) .*/\1/' "/spiflash/image_process.txt.md5"
     sprintf(command, "md5sum %s > %s && sed -i 's/^\([^ ]* [^ ]*\) .*/\1/' %s", filedata, filedatamd5,filedatamd5);

     printf("%s",command);
     // 执行shell命令
     system(command);

}
#endif
int  exectProgrammd5test(char *filedata,char *filedatamd5)
{

   char result[2048] = {0};
   unsigned int exectcount=0;
    // 检查文件路径是否存在和是否有读取权限
    if (access(filedata, R_OK) != 0) {
        printf("File does not exist or cannot be read: %s\n", filedata);
        return -1;
    }

    // 构建shell命令
    char command[512];
    //md5sum /mnt/spiflashtxt/image_process>/mnt/spiflashtxt/imageprocess.txt.md5 &&sed -i 's/ .*//' /mnt/spiflashtxt/imageprocess.txt.md5
    // md5sum /mnt/spiflashtxt/image_process>/mnt/spiflashtxt/imageprocess.txt.md5 &&sed -i 's/^\([^ ]* [^ ]*\) .*/\1/' "/mnt/spiflashtxt/imageprocess.txt.md5"
    snprintf(command, sizeof(command), "md5sum %s > %s &&sed -i 's/ .*//' %s", filedata, filedatamd5, filedatamd5);

    //printf("%s\n", command);

    // 执行shell命令
    int ret = system(command);
    if (ret != 0) {
        #ifdef IF_PRINTF_MONITOR
        printf("Command execution failed: %s\n", command);
        #endif
        return ret;
    }


   #if 0
   ret=ExecuteCMD("cp /spiflash/image_process  /spiflash/usr_process_1",result);
   if(ret!=0)
   {
       return ret;
   }
   ret=ExecuteCMD("cp /spiflash/image_process  /spiflash/usr_process_2",result);
   if(ret!=0)
   {
       return ret;
   }
   ret=ExecuteCMD("cp /spiflash/image_process  /spiflash/usr_process_3",result);
   if(ret!=0)
   {
       return ret;
   }
   #endif

   ret=ExecuteCMD("cp /mnt/spiflashtxt/image_process  /mnt/spiflashtxt/usr_process_1",result);
   if(ret==0)
   {
        exectcount++;
   }

   ret=ExecuteCMD("cp /mnt/spiflashtxt/imageprocess.txt.md5  /mnt/spiflashtxt/usr_process_1",result);
   if(ret==0)
   {
        exectcount++;
   }
   ret=ExecuteCMD("cp /mnt/spiflashtxt/image_process  /mnt/spiflashtxt/usr_process_2",result);
   if(ret==0)
   {
       exectcount++;
   }

   ret=ExecuteCMD("cp /mnt/spiflashtxt/imageprocess.txt.md5  /mnt/spiflashtxt/usr_process_2",result);
   if(ret==0)
   {
        exectcount++;
   }
   ret=ExecuteCMD("cp /mnt/spiflashtxt/image_process  /mnt/spiflashtxt/usr_process_3",result);
   if(ret==0)
   {
       exectcount++;
   }
   ret=ExecuteCMD("cp /mnt/spiflashtxt/imageprocess.txt.md5  /mnt/spiflashtxt/usr_process_3",result);
   if(ret==0)
   {
        exectcount++;
   }


   if(exectcount==6)
   {
       ret=0;
   }
   else
   {
       ret=-1;
   }

   //printf("count %d\n",exectcount);
   return ret;
}




int  exectProgrammd5(char *filedata,char *filedatamd5)
{

   char result[2048] = {0};
   unsigned int exectcount=0;
    // 检查文件路径是否存在和是否有读取权限
    if (access(filedata, R_OK) != 0) {
        #ifdef IF_PRINTF_MONITOR
        printf("File does not exist or cannot be read: %s\n", filedata);
        #endif
        return -1;
    }

    // 构建shell命令
    char command[512];
    //md5sum /mnt/spiflashtxt/image_process>/mnt/spiflashtxt/imageprocess.txt.md5 &&sed -i 's/ .*//' /mnt/spiflashtxt/imageprocess.txt.md5
    // md5sum /mnt/spiflashtxt/image_process>/mnt/spiflashtxt/imageprocess.txt.md5 &&sed -i 's/^\([^ ]* [^ ]*\) .*/\1/' "/mnt/spiflashtxt/imageprocess.txt.md5"
    snprintf(command, sizeof(command), "md5sum %s > %s &&sed -i 's/ .*//' %s", filedata, filedatamd5, filedatamd5);

    //printf("%s\n", command);

    // 执行shell命令
    int ret = system(command);
    if (ret != 0) {
        #ifdef IF_PRINTF_MONITOR
        printf("Command execution failed: %s\n", command);
        #endif
        return ret;
    }


   #if 0
   ret=ExecuteCMD("cp /spiflash/image_process  /spiflash/usr_process_1",result);
   if(ret!=0)
   {
       return ret;
   }
   ret=ExecuteCMD("cp /spiflash/image_process  /spiflash/usr_process_2",result);
   if(ret!=0)
   {
       return ret;
   }
   ret=ExecuteCMD("cp /spiflash/image_process  /spiflash/usr_process_3",result);
   if(ret!=0)
   {
       return ret;
   }
   #endif

   ret=ExecuteCMD("cp /usr/bin/image_process  /spiflash/usr_process_1",result);
   if(ret==0)
   {
       exectcount++;
   }
   ret=ExecuteCMD("cp /usr/bin/imageprocess.txt.md5  /spiflash/usr_process_1",result);
   if(ret==0)
   {
        exectcount++;
   }
   ret=ExecuteCMD("cp /usr/bin/image_process  /spiflash/usr_process_2",result);
   if(ret==0)
   {
       exectcount++;
   }
   ret=ExecuteCMD("cp /usr/bin/imageprocess.txt.md5  /spiflash/usr_process_2",result);
   if(ret==0)
   {
        exectcount++;
   }
   ret=ExecuteCMD("cp /usr/bin/image_process  /spiflash/usr_process_3",result);
   if(ret==0)
   {
      exectcount++;
   }
   ret=ExecuteCMD("cp /usr/bin/imageprocess.txt.md5  /spiflash/usr_process_3",result);
   if(ret==0)
   {
        exectcount++;
   }

   if(exectcount==6)
   {
       ret=0;
   }
   else
   {
       ret=-1;
   }

   //printf("exectcount:%d\n",exectcount);
   return ret;
}

#if 0
int exectProgrammd5(char *filedata, char *filedatamd5)
{
    // 检查文件路径是否存在和是否有读取权限
    if (access(filedata, R_OK) != 0) {
        printf("File does not exist or cannot be read: %s\n", filedata);
        return 0;
    }

    // 构建shell命令
    char command[512];
    snprintf(command, sizeof(command), "md5sum %s > %s && sed -i 's/ .*//' %s", filedata, filedatamd5, filedatamd5);

    printf("%s\n", command);

    // 执行shell命令
    FILE *fp = popen(command, "r");
    if (fp == NULL) {
        printf("Failed to execute command: %s\n", command);
        return 0;
    }

    // 读取命令输出
    char output[512];
    while (fgets(output, sizeof(output), fp) != NULL) {
        // 处理命令输出
        printf("%s", output);
    }

    // 关闭文件指针
    pclose(fp);

    return 1;
}
#endif

/**
 * @brief （已完成）在轨编程完全实施
 *
 * @param route_log
 * @param n
 * @return int 完成替换返回0，替换失败返回1
 */
//int executeOnlineProgram(const char route_log[], int n)
int executeOnlineProgram()
{
    // //获取当前的在轨编程状态
    // int ret = getLog_All(LOG_ROUTE, sizeof(LOG_ROUTE), &log_data);
    int ret;
    char result[2048] = {0};

    //读取文件进行文件拆包
    FILE *fd_first_pack = fopen(FIRST_PACK_ROUTE, "rb"); //文件IO设置为只读模式
    if (fd_first_pack == NULL)
    {
        #ifdef  IF_PRINTF_MONITOR
        printf("processReceivedPack: the %s doesn't exist! \n", FIRST_PACK_ROUTE);
        #endif
        return -1;
    }
    fseek(fd_first_pack, 0, SEEK_END);
    //查询现在有多少文件
    long lenth=ftell(fd_first_pack);
    fd_first_pack_lenth=lenth-12;
    fseek(fd_first_pack, 8, SEEK_SET);
    fread(data_zh,lenth-12,1,fd_first_pack);
    createLibraryFileOut(LIB_SAVE_DIR, sizeof(LIB_SAVE_DIR),data_zh,fd_first_pack_lenth);

    //拆成小包后将其替换到该存放的位置


    //完成替换

   // ret=ExecuteCMD("cp -r /mnt/logzg/lib_dir/.  /mnt",result);
    //ret=ExecuteCMD("cp -r /spiflash/logzg/lib_dir/.  /spiflash",result);   //2023.12.27换到了/usr/bin更改
    ret=ExecuteCMD("cp -r /spiflash/logzg/lib_dir/.  /usr/bin",result);

    //ret=ExecuteCMD("cp -r /logzg/lib_dir/.  /logzg_backup",result);
    //ret=ExecuteCMD("cp -r /spiflash/logzg/lib_dir/.  /spiflash",result);
    if(ret == 0)
    {

        log_data.flag_update=0xff;

        #ifdef  IF_PRINTF_MONITOR
        printf("executeOnlineProgram: Backup complete! \n");
        #endif
    }
    else
    {
        #ifdef  IF_PRINTF_MONITOR
        printf("executeOnlineProgram: Backup failed! \n");
        #endif
        return -1;
    }


    //计算本文件夹的md5值  2023.12.5
    //exectProgrammd5("/spiflash/", char *filedatamd5)

    //ret=exectProgrammd5("/spiflash/image_process","/spiflash/imageprocess.txt.md5");  //2023.12.27换到了/usr/bin更改
    ret=exectProgrammd5((char*)"/usr/bin/image_process",(char*)"/usr/bin/imageprocess.txt.md5");



    //（待修改）是否此时就将log更新进log文件？
    setLog_All(LOG_ROUTE, sizeof(LOG_ROUTE), &log_data);
    #ifdef  IF_PRINTF_MONITOR
    printf("executeOnlineProgram: Replace file complete! \n");
    #endif


    if(ret==0)  //成功
     {
       return 0;
    }
    else
    {
        return -1;
    }
    //return 0;
}
