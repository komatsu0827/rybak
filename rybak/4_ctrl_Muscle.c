// ------------------------------------ //
//	�l�H�ؓ�����p�v���O����	//
//	ctrl_Muscle.c			//
//	(control_Muscle)		//
// ------------------------------------ //


#include "8_define.h"
#include "9_user_param.h"


typedef struct {
	int DIOorPWM;
	int pin;
} pinAssign;

//�d���ِ���p�\����
pinAssign pin_list[PINS] = {
	//LF
{DIO, 0},	// LF	1�ԋ�	0
{DIO, 1},	// LF	2�ԋ�	1
{DIO, 2},	// LF	3�ԋ�	2
{DIO, 3},	// LF	4�ԋ�	3
{DIO, 4},	// LF	5�ԋ�	4
{DIO, 5},	// LF	6�ԋ�	5

//LH
{DIO, 8},	   // LH	1�ԋ�	0
{DIO, 9},	  // LH	2�ԋ�	1
{DIO, 10},	// LH	3�ԋ�	2
{DIO, 11},	// LH	4�ԋ�	3
{DIO, 12},	// LH	5�ԋ�	4
{DIO, 13},	// LH	6�ԋ�	5
//RF
{DIO, 16},	//RF	1�ԋ�	0
{DIO, 17},	// RF	2�ԋ�	1
{DIO, 18},	// RF	3�ԋ�	2
{DIO, 19},	// RF	4�ԋ�	3
{DIO, 20},	// RF	5�ԋ�	4
{DIO, 21},	// RF	6�ԋ�	5
//RH
{DIO, 24},	// RH	1�ԋ�	0
{DIO, 25},	// RH	2�ԋ�	1
{DIO, 26},	// RH	3�ԋ�	2
{DIO, 27},	// RH	4�ԋ�	3
{DIO, 28},	// RH	5�ԋ�	4
{DIO, 29},	// RH	6�ԋ�	5
};

float pre_f[MUSCLES_PER_LEG] = { 0.0 };
float f_max[6] = {1.0,1.0,0.75,1.0,0.5,0.9};	//�˕�����̏C�_�ł̋ؓ����̊���Q�C��

//Mn�U������
float Mn_addition[UNITS][MUSCLES_PER_LEG] = {{0.0}}; //Mn�̒l�����Z
int Mn_add_count[UNITS][MUSCLES_PER_LEG] = {{0}};   //Mn�̉��Z��
int valve_count[UNITS][MUSCLES_PER_LEG] = {{0}};    //�d���ق̋쓮��
int valve_flag[UNITS][MUSCLES_PER_LEG] = {{OFF}}; //Mn�����΂��Ă��邩
float t_last_valve[UNITS][MUSCLES_PER_LEG] = {{0.0}}; //�d���ً쓮�̍ŏI���s����

/*--------------------------------------*/
/*		�ق�ON/OFF����		*/
/*		set_valve()		*/
/*--------------------------------------*/
void set_valve(int x, int ONOFF)
{
	if (x < 0 || PINS <= x) return;

	if (pin_list[x].DIOorPWM == DIO)
		setDIO(pin_list[x].pin, ONOFF);
	else if (pin_list[x].DIOorPWM == PWM)
		setPWMduty(pin_list[x].pin, (float)ONOFF);
}

/*----------------------------------------------*/
/*		�l�H�ؓ��̏�Ԑ���		*/
/*		setValves()			*/
/*----------------------------------------------*/
void setValves(int i, int muscle, int state)
{
	if (state == ON){		// ���k
		set_valve(i * 6 + muscle, ON);
		//set_valve(i * 12 + muscle * 2 + 1, OFF);
	}
	else if (state == OFF) {	// �o��
		set_valve(i * 6 + muscle, OFF);
		//set_valve(i * 12 + muscle * 2 + 1, ON);
	}
	/*else if (state == KEEP) {	// �ێ�
		set_valve(i * 12 + muscle * 2, OFF);
		set_valve(i * 12 + muscle * 2 + 1, OFF);
	}*/
    muscle_onoff(i, muscle, state);

}

//----------------------------------------------//
//	�^���j���[�����̔��Ώ�Ԃ���ؓ��𐧌�      	//
//		Mn_addtion()	   Mn�̒l���擾         //
//		Mn_average()	   Mn�̕��ϒl���Z�o	  //
//		control_Muscles()	�d���ق̋쓮	       //
//----------------------------------------------//
//Mn�̒l�̉��Z�Ǝ擾��  ���ώZ�o�p & Mn�̌��݂̒l��Ԃ�
float Mn_Addition(int i, int j, RybakUnit Unit[UNITS])
{
    float Mn_V;

    Mn_V = GetfV(Unit[i].Neuron_V[Mn_1+j], Mn_1+j);
    Mn_addition[i][j] += Mn_V;  //Mn�����Z
    Mn_add_count[i][j] ++;  //�񐔂̃J�E���g

    return Mn_V;    //���݂̒l��Ԃ�
}

//Mn�̕��ϒl���Z�o�C�d���ق̊J����������   �d���ِ���3��
int Mn_Average(int i,int j)
{

    float Mn_average;
    int mode;

    Mn_average = Mn_addition[i][j] / Mn_add_count[i][j]; //Mn�̕��ϒl���Z�o
    if (Mn_average < Mn_phase0 )
        mode = 0;   //mode0 [0 0 0]
    else if (Mn_average < Mn_phase1 )
        mode = 1;   //mode1 [1 0 0]
    else if (Mn_average < Mn_phase2 )
        mode = 2;   //mode2 [1 1 0]
    else
        mode = 3;   //mode3 [1 1 1]

    return mode;
}

//�d���ِ���   2ms��
void control_Muscles(float t, RybakUnit Unit[UNITS])
{
	int i, j;

    for(i=0; i<UNITS; i++){
        for(j=0; j<MUSCLES_PER_LEG; j++){

            //����1�FMn�����΂��Ă��Ȃ��Ƃ�
            if(valve_flag[i][j] == OFF){
                //Mn�����΂�����d����ON & ����2��
                if(GetfV(Unit[i].Neuron_V[Mn_1+j], Mn_1+j)>=Mn_fVth){
                    valve_mode[i][j] = 3;
                    valve_count[i][j] = 0;
                    valve_flag[i][j] = ON;
                }
            }

            //����2�FMn�����΂��Ă���Ƃ�
            if(valve_flag[i][j] == ON){
                if(Mn_Addition(i, j, Unit) < Mn_fVth){   //Mn�����Z(2ms��)
                    valve_mode[i][j] = 0;
                    setValves(i, j, OFF);
                    valve_flag[i][j] = OFF;
                    //Mn���Z�l�C���Z�񐔂̃��Z�b�g
                    Mn_addition[i][j] = 0.0;
                    Mn_add_count[i][j] = 0;

                    break;
                }

				if(j==0 || j==1){
					if(GetfV(Unit[i].Neuron_V[Mn_1+j], Mn_1+j)>=0.15){
						valve_mode[i][j] = 3;
						valve_count[i][j] = 0;
					}
				}

                //17ms��
                if(1.7e-2 < t - t_last_valve[i][j]){

                    if (valve_count[i][j] == 3){    //3��(51ms)���Ɏ��s
                        valve_mode[i][j] = Mn_Average(i,j);    //Mn���ϒl����d���ك��[�h�X�V
                        //Mn���Z�l�C���Z�񐔁C���s��(17ms��)�̃��Z�b�g
                        Mn_addition[i][j] = 0.0;
                        Mn_add_count[i][j] = 0;
                        valve_count[i][j] = 0;
                    }

                    if(valve_mode[i][j] == 0)	//�d���يJ���� mode0�̎�
                        setValves(i, j, OFF);	// count(0,1,2)��OFF

                    else if(valve_mode[i][j] == 1){	//�d���يJ���� mode1�̎�
                        if(valve_count[i][j] == 0)
                            setValves(i, j, ON);	// count(0)��ON
                        else
                            setValves(i, j, OFF);	// count(1,2)��OFF
                    }
                    else if(valve_mode[i][j] == 2){	//�d���يJ���� mode2�̎�
                        if(valve_count[i][j] == 0 || valve_count[i][j] == 1)
                            setValves(i, j, ON);	// count(0,1)��ON
                        else
                            setValves(i, j, OFF);	// count(2)��OFF
                    }
                    else
                        setValves(i, j, ON);	//�d���يJ���� mode3�̎�

                    valve_count[i][j] ++;
                    t_last_valve[i][j] = t;
                }


            }


        }
    }


}

//--------------------------------------------------------------//
//		�ؓ��̋�������(CPG�̏�Ԃɂ��Ȃ�)		//
//	muscle_flexion(), muscle_extension(), muscle_reset()	//
//--------------------------------------------------------------//
//�r�����Ȃ�����
void muscle_flexion() {
	int	i;
//	float	t = 0.0;
//	float	ts, tn;
//	double	tmu_frq = tsc_clock();
//	ts = tn = rdtsc();
//	while(t < Time_Flexion) {
//		tn = rdtsc();
//		t = (tn-ts) / tmu_frq / 1.0e6;
		for(i=0; i < UNITS; i++) {
			setValves(i, 0, ON);
			setValves(i, 1, OFF);
			setValves(i, 2, ON);
			setValves(i, 3, OFF);
			setValves(i, 4, ON);
			setValves(i, 5, OFF);
		}
//	}
}

//�r��L�W������
void muscle_extension(){
	int	i;
//	float	t = 0.0;
//	float	ts, tn;
//	double	tmu_frq = tsc_clock();
//	ts = tn = rdtsc();
//	while(t < Time_Extension) {
//		tn = rdtsc();
//		t = (tn-ts) / tmu_frq / 1.0e6;
		for(i=0; i < UNITS; i++) {
			setValves(i, 0, OFF);
			setValves(i, 1, ON);
			setValves(i, 2, OFF);
			setValves(i, 3, ON);
			setValves(i, 4, OFF);
			setValves(i, 5, ON);
		}
//	}
}

//�S�ؓ���o�ɂ�����
void muscle_reset(){
	int	i;
//	float	t = 0.0;
//	float	ts, tn;
//	double	tmu_frq = tsc_clock();
//	ts = tn = rdtsc();
//	while(t < Time_Reset) {
//		tn = rdtsc();
//		t = (tn-ts) / tmu_frq / 1.0e6;
		for(i=0; i < UNITS; i++) {
			setValves(i, 0, OFF);
			setValves(i, 1, OFF);
			setValves(i, 2, OFF);
			setValves(i, 3, OFF);
			setValves(i, 4, OFF);
			setValves(i, 5, OFF);
		}
//	}
}



//�d���ق�ONOFF���L�^
void muscle_onoff(int unit, int muscle, int ONOFF)
{
    unsigned int shift_flag = 0;
    unsigned int flag_bit = 1;

    shift_flag = flag_bit << (unit*6 + muscle);
    muscle_io |= shift_flag;    //�Ώۂ̋r�̕�����1�ɂ���

    if(ONOFF == OFF){
        muscle_io ^= shift_flag;    //�d���ق�OFF�̂Ƃ��͔��](1��0)
    }

}