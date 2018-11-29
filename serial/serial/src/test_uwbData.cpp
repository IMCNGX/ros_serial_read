#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix 标准函数定义*/
#include     <sys/types.h>  
#include     <sys/stat.h>   
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "serial/uwb.h"

#define FALSE -1
#define TRUE 0
#define int16 short
#define uint16 unsigned short

int16 x;
int16 y;
int16 yaw;
int16 dis[6]={0};
int16 error;
int16 keep;

int i=0;
int j=0;

int main(int argc, char **argv)
{
	unsigned char buf[300];
	int t = 0;
	int k = 0;
	int fd,flag_close,retv;
	struct  termios opt;
	
	ros::init(argc, argv, "uwb");
	ros::NodeHandle n;
	ros::Publisher uwb_pub = n.advertise<serial::uwb>("uwb", 1000);
	ros::Rate loop_rate(50);

	fd = open("/dev/ttyACM0",O_RDWR);
	if(fd==-1)
	{
		fd = open("/dev/ttyACM1",O_RDWR);
		if(fd==-1)
		{	
			fd = open("/dev/ttyACM2",O_RDWR);
			if(fd==-1)
			{		
				printf("open failed\n");
				exit (0);
			}
		}
	}
	else
		printf("open success\n");
	tcgetattr(fd,&opt);
	cfsetispeed(&opt,B115200); 
	cfsetospeed(&opt,B115200);
	
	opt.c_cflag |= (CLOCAL | CREAD);                    
	// 无校验 8位数据位1位停止位
	opt.c_cflag &= ~PARENB;                         
	opt.c_cflag &= ~CSTOPB;
	opt.c_cflag &= ~CSIZE; 
	// 8个数据位
	opt.c_cflag |= CS8; 
	// 原始数据输入
	opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
	opt.c_oflag &= ~(OPOST);
	// 设置等待时间和最小接收字符数
	opt.c_cc[VTIME]    = 0;                          
	opt.c_cc[VMIN]     = 0;   
	// 处理未接收的字符
	tcflush(fd, TCIFLUSH);  
	tcsetattr(fd,TCSANOW,&opt);                         
	printf("start send\n");
	//printf("receive data is:");
	while(ros::ok())
	{		
		serial::uwb msg;

		retv=read(fd,buf,300);   
		if(retv==-1)	
			printf("read failed\n");
		if(retv>0)
		{
			printf("%d",retv);
			for(k=0;k<retv;k++)
			{		
				switch(k)
				{
					case 0: printf("|x is");
						printf(" %04d",x=buf[k]+(buf[k+1]<<8));
						//msg.x=x;
						msg.x=374-i;
						i++;
						if(msg.x<=0)
						{
						    msg.x=0;
						}					
						break;
					case 2:	printf("|y is");
						printf(" %04d",y=buf[k]+(buf[k+1]<<8));
						//msg.y=y;
						msg.y=74-j;
						j++;
						if(msg.y<=0)
						{
						    msg.y=0;
						}
						break;
					case 4: printf("|yaw is");
						printf(" %04d",yaw=buf[k]+(buf[k+1]<<8));
						msg.yaw=yaw;
						break;
					case 6: printf("|d0 is");
						printf(" %04d",dis[0]=buf[k]+(buf[k+1]<<8));
						msg.d0=dis[0];
						break;
					case 8: printf("|d1 is");
						printf(" %04d",dis[1]=buf[k]+(buf[k+1]<<8));
						msg.d1=dis[1];
						break;
					case 10: printf("|d2 is");
						 printf(" %04d",dis[2]=buf[k]+(buf[k+1]<<8));
						 msg.d2=dis[2];
						 break;
					case 12: printf("|d3 is");
						 printf(" %04d",dis[3]=buf[k]+(buf[k+1]<<8));
						 msg.d3=dis[3];
						 break;
					case 14: printf("|d4 is");
						 printf(" %04d",dis[4]=buf[k]+(buf[k+1]<<8));
						 msg.d4=dis[4];
						 break;
					case 16: printf("|d5 is");
						 printf(" %04d",dis[5]=buf[k]+(buf[k+1]<<8));
						 msg.d5=dis[5];
						 break;
					case 18: printf("|error is");
						 printf(" %04d",error=buf[k]+(buf[k+1]<<8));
						 msg.error=error;
						 break;
					case 20: printf("|keep is");
						 printf(" %04d\n ",keep=buf[k]+(buf[k+1]<<8));
						 msg.keep=keep;
						 break;
				}
			}
			uwb_pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
		}

	}
	usleep(20);		 
	flag_close = close(fd);
	if(flag_close == -1)   
		printf("Close the Device failed\n");
	return 0;
}
