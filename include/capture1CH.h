#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/ethernet.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <linux/if_arp.h>
//#include <linux/in.h>
#include <linux/sockios.h>
#include <time.h> 
#include <unistd.h> 
//#include <pthread.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <arpa/inet.h>

#define DEFAULT_IF "eth0"
#define IFNAMESIZ 		32	
#define AVTP_PAYLOAD_OFFSET  	54
#define AVTP_PAYLOAD_SIZE       992
#define ETH_FRAME_LEN			1500
#define ETHERTYPE_AVTP			0x22f0
// #define MTU_SIZE			1500
#define QUEUE_SIZE  			1024
#define PACKET_LIMIT			1000

#define NEXT(index)   ( (index + 1) % QUEUE_SIZE ) //원형 큐에서 인덱스를 변경하는 매크로 함수

static const int rec_max_size = 1000*1500;

// init for Socket
int avb_socket;
struct ifreq ifr;						
struct sockaddr_ll socketAddress;		
bool boolIsEnabled = false;

// 0: stacking files, 1: save video
int thr_id_0, thr_id_1, a=1, status=1;
//std::thread* p_thread[2]; // pthread_t p_thread[2];

// buffer
char temp_buf[1500] = {0}; // MTU_SIZE
char *recv_buffer[3] = {NULL, }; // stacked packets
int recv_size=0, rec_written[3]={0,};
int nrPkt=0, nrContainer=0, nrVideo=0;
std::mutex gmutex;
std::condition_variable gcond;
// pthread_mutex_t gmutex; pthread_cond_t gcond;


// output video 
FILE *f_vid;
char path2video[100], imgDir[200];
int nCapturedImage = 1;

int socketOpen();
void threadFcn0();
void threadFcn1();
//void* threadFcn0(void *data);
//void* threadFcn1(void *data);
