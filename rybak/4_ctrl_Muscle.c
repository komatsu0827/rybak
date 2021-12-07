// ------------------------------------ //
//	人工筋肉制御用プログラム	//
//	ctrl_Muscle.c			//
//	(control_Muscle)		//
// ------------------------------------ //


#include "8_define.h"
#include "9_user_param.h"


typedef struct {
	int DIOorPWM;
	int pin;
} pinAssign;

//電磁弁制御用構造体
pinAssign pin_list[PINS] = {
	//LF
{DIO, 0},	// LF	1番筋	0
{DIO, 1},	// LF	2番筋	1
{DIO, 2},	// LF	3番筋	2
{DIO, 3},	// LF	4番筋	3
{DIO, 4},	// LF	5番筋	4
{DIO, 5},	// LF	6番筋	5

//LH
{DIO, 8},	   // LH	1番筋	0
{DIO, 9},	  // LH	2番筋	1
{DIO, 10},	// LH	3番筋	2
{DIO, 11},	// LH	4番筋	3
{DIO, 12},	// LH	5番筋	4
{DIO, 13},	// LH	6番筋	5
//RF
{DIO, 16},	//RF	1番筋	0
{DIO, 17},	// RF	2番筋	1
{DIO, 18},	// RF	3番筋	2
{DIO, 19},	// RF	4番筋	3
{DIO, 20},	// RF	5番筋	4
{DIO, 21},	// RF	6番筋	5
//RH
{DIO, 24},	// RH	1番筋	0
{DIO, 25},	// RH	2番筋	1
{DIO, 26},	// RH	3番筋	2
{DIO, 27},	// RH	4番筋	3
{DIO, 28},	// RH	5番筋	4
{DIO, 29},	// RH	6番筋	5
};

float pre_f[MUSCLES_PER_LEG] = { 0.0 };
float f_max[6] = {1.0,1.0,0.75,1.0,0.5,0.9};	//戸部さんの修論での筋肉毎の既定ゲイン

//Mn振幅制御
float Mn_addition[UNITS][MUSCLES_PER_LEG] = {{0.0}}; //Mnの値を加算
int Mn_add_count[UNITS][MUSCLES_PER_LEG] = {{0}};   //Mnの加算回数
int valve_count[UNITS][MUSCLES_PER_LEG] = {{0}};    //電磁弁の駆動回数
int valve_flag[UNITS][MUSCLES_PER_LEG] = {{OFF}}; //Mnが発火しているか
float t_last_valve[UNITS][MUSCLES_PER_LEG] = {{0.0}}; //電磁弁駆動の最終実行時間

/*--------------------------------------*/
/*		弁のON/OFF制御		*/
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
/*		人工筋肉の状態制御		*/
/*		setValves()			*/
/*----------------------------------------------*/
void setValves(int i, int muscle, int state)
{
	if (state == ON){		// 収縮
		set_valve(i * 6 + muscle, ON);
		//set_valve(i * 12 + muscle * 2 + 1, OFF);
	}
	else if (state == OFF) {	// 弛緩
		set_valve(i * 6 + muscle, OFF);
		//set_valve(i * 12 + muscle * 2 + 1, ON);
	}
	/*else if (state == KEEP) {	// 保持
		set_valve(i * 12 + muscle * 2, OFF);
		set_valve(i * 12 + muscle * 2 + 1, OFF);
	}*/
    muscle_onoff(i, muscle, state);

}

//----------------------------------------------//
//	運動ニューロンの発火状態から筋肉を制御      	//
//		Mn_addtion()	   Mnの値を取得         //
//		Mn_average()	   Mnの平均値を算出	  //
//		control_Muscles()	電磁弁の駆動	       //
//----------------------------------------------//
//Mnの値の加算と取得回数  平均算出用 & Mnの現在の値を返す
float Mn_Addition(int i, int j, RybakUnit Unit[UNITS])
{
    float Mn_V;

    Mn_V = GetfV(Unit[i].Neuron_V[Mn_1+j], Mn_1+j);
    Mn_addition[i][j] += Mn_V;  //Mnを加算
    Mn_add_count[i][j] ++;  //回数のカウント

    return Mn_V;    //現在の値を返す
}

//Mnの平均値を算出，電磁弁の開放率を決定   電磁弁制御3回毎
int Mn_Average(int i,int j)
{

    float Mn_average;
    int mode;

    Mn_average = Mn_addition[i][j] / Mn_add_count[i][j]; //Mnの平均値を算出
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

//電磁弁制御   2ms毎
void control_Muscles(float t, RybakUnit Unit[UNITS])
{
	int i, j;

    for(i=0; i<UNITS; i++){
        for(j=0; j<MUSCLES_PER_LEG; j++){

            //分岐1：Mnが発火していないとき
            if(valve_flag[i][j] == OFF){
                //Mnが発火したら電磁弁ON & 分岐2へ
                if(GetfV(Unit[i].Neuron_V[Mn_1+j], Mn_1+j)>=Mn_fVth){
                    valve_mode[i][j] = 3;
                    valve_count[i][j] = 0;
                    valve_flag[i][j] = ON;
                }
            }

            //分岐2：Mnが発火しているとき
            if(valve_flag[i][j] == ON){
                if(Mn_Addition(i, j, Unit) < Mn_fVth){   //Mnを加算(2ms毎)
                    valve_mode[i][j] = 0;
                    setValves(i, j, OFF);
                    valve_flag[i][j] = OFF;
                    //Mn加算値，加算回数のリセット
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

                //17ms毎
                if(1.7e-2 < t - t_last_valve[i][j]){

                    if (valve_count[i][j] == 3){    //3回(51ms)毎に実行
                        valve_mode[i][j] = Mn_Average(i,j);    //Mn平均値から電磁弁モード更新
                        //Mn加算値，加算回数，実行回数(17ms毎)のリセット
                        Mn_addition[i][j] = 0.0;
                        Mn_add_count[i][j] = 0;
                        valve_count[i][j] = 0;
                    }

                    if(valve_mode[i][j] == 0)	//電磁弁開放率 mode0の時
                        setValves(i, j, OFF);	// count(0,1,2)でOFF

                    else if(valve_mode[i][j] == 1){	//電磁弁開放率 mode1の時
                        if(valve_count[i][j] == 0)
                            setValves(i, j, ON);	// count(0)でON
                        else
                            setValves(i, j, OFF);	// count(1,2)でOFF
                    }
                    else if(valve_mode[i][j] == 2){	//電磁弁開放率 mode2の時
                        if(valve_count[i][j] == 0 || valve_count[i][j] == 1)
                            setValves(i, j, ON);	// count(0,1)でON
                        else
                            setValves(i, j, OFF);	// count(2)でOFF
                    }
                    else
                        setValves(i, j, ON);	//電磁弁開放率 mode3の時

                    valve_count[i][j] ++;
                    t_last_valve[i][j] = t;
                }


            }


        }
    }


}

//--------------------------------------------------------------//
//		筋肉の強制制御(CPGの状態によらない)		//
//	muscle_flexion(), muscle_extension(), muscle_reset()	//
//--------------------------------------------------------------//
//脚を屈曲させる
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

//脚を伸展させる
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

//全筋肉を弛緩させる
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



//電磁弁のONOFFを記録
void muscle_onoff(int unit, int muscle, int ONOFF)
{
    unsigned int shift_flag = 0;
    unsigned int flag_bit = 1;

    shift_flag = flag_bit << (unit*6 + muscle);
    muscle_io |= shift_flag;    //対象の脚の部分を1にする

    if(ONOFF == OFF){
        muscle_io ^= shift_flag;    //電磁弁がOFFのときは反転(1→0)
    }

}
