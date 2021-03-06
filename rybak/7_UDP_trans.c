// ------------------------------------- //
//	UDP通信（送信側）用関数		//
//	UDP_trans.c			//
//	(UDP_transmission)		//
// ------------------------------------ //

#include "8_define.h"
#include "9_user_param.h"

//UDP通信用
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


//エアトランスホーマ_0.6MPaG
//前脚
#define	Pvalve_duty_1_F	(0.83) //(0.75)//(0.83)//0.83	//比例弁_初期duty比
#define	Pvalve_duty_2_F	(0.83) //(0.75)//(1.0)	//比例弁_変更後のduty比
//後脚 ***前脚と同じ***
#define	Pvalve_duty_1_H	(Pvalve_duty_1_F)//(0.83)//0.83	//比例弁_初期duty比
#define	Pvalve_duty_2_H	(Pvalve_duty_2_F)//(1.0)	//比例弁_変更後のduty比

void udp_transmission(float t)
{
	//------UDP----------------------------------------------------------------------------
	int	sock;
	struct	sockaddr_in	addr;
	double	SendData[2][2] = { {1.0, 0.0},
				   {1.0, 0.0} };	//{duty, fin_flag}

	sock = socket(AF_INET, SOCK_DGRAM, 0);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(8008);	//
	addr.sin_addr.s_addr = inet_addr("192.168.11.84");	//送信先HRPのIP(84:kokura)
	//-------------------------------------------------------------------------------------

	//初期1次気圧用data送信(受信側でduty比指定して駆動させておく場合不要)
	//SendData[0] = 1.0;	//duty比
	//sendto(sock, SendData, sizeof(SendData), 0, (struct sockaddr *)&addr, sizeof(addr));

	//運動中 比例弁duty比変更
	if(t >= Time_Step4) {
		if(t <= Time_State1) {
			SendData[0][0] = Pvalve_duty_1_F;
			SendData[1][0] = Pvalve_duty_1_H;
		}

		else if(t < Time_State2) {
			SendData[0][0] = Pvalve_duty_1_F + (Pvalve_duty_2_F - Pvalve_duty_1_F) / (Time_State2 - Time_State1) * (t - Time_State1);
			SendData[1][0] = Pvalve_duty_1_H + (Pvalve_duty_2_H - Pvalve_duty_1_H) / (Time_State2 - Time_State1) * (t - Time_State1);
		}

		else if(t < Time_Step5) {
			SendData[0][0] = Pvalve_duty_2_F;
			SendData[1][0] = Pvalve_duty_2_H;
		}
		else {
			SendData[0][1] = 1.0;	//fin_flag
			SendData[1][1] = 1.0;	//fin_flag
		}
	}
	//UDP_送信
	sendto(sock, SendData, sizeof(SendData), 0, (struct sockaddr *)&addr, sizeof(addr));

	close(sock);
}
