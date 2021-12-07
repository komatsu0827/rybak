// ------------------------------------- //
//	UDP�ʐM�i���M���j�p�֐�		//
//	UDP_trans.c			//
//	(UDP_transmission)		//
// ------------------------------------ //

#include "8_define.h"
#include "9_user_param.h"

//UDP�ʐM�p
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


//�G�A�g�����X�z�[�}_0.6MPaG
//�O�r
#define	Pvalve_duty_1_F	(0.83) //(0.75)//(0.83)//0.83	//����_����duty��
#define	Pvalve_duty_2_F	(0.83) //(0.75)//(1.0)	//����_�ύX���duty��
//��r ***�O�r�Ɠ���***
#define	Pvalve_duty_1_H	(Pvalve_duty_1_F)//(0.83)//0.83	//����_����duty��
#define	Pvalve_duty_2_H	(Pvalve_duty_2_F)//(1.0)	//����_�ύX���duty��

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
	addr.sin_addr.s_addr = inet_addr("192.168.11.84");	//���M��HRP��IP(84:kokura)
	//-------------------------------------------------------------------------------------

	//����1���C���pdata���M(��M����duty��w�肵�ċ쓮�����Ă����ꍇ�s�v)
	//SendData[0] = 1.0;	//duty��
	//sendto(sock, SendData, sizeof(SendData), 0, (struct sockaddr *)&addr, sizeof(addr));

	//�^���� ����duty��ύX
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
	//UDP_���M
	sendto(sock, SendData, sizeof(SendData), 0, (struct sockaddr *)&addr, sizeof(addr));

	close(sock);
}