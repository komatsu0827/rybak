// -------------------------------------------- //
//	locomotionに関与していない関数群	//
//	function.h				//
// -------------------------------------------- //

#include "8_define.h"
#include "9_user_param.h"


/*--------------------------------------*/
/*	各サンプルデータを配列に格納	*/
/*	data_get()			*/
/*--------------------------------------*/
void data_get(int k, float t, SAMType *sam, RybakUnit Unit[], ENCType *enc, ADType *ad, LOADType *load)
{
	/*------変数宣言------*/
	int i;	//カウンタ


	//時間の保存
	sam->t[k] = t - Time_Step4;	// 歩行開始時刻を0[s]とする

	for(i=0; i<UNITS; i++){
		//CPG周期の保存
		sam->cpg_cycle[i][k] = Unit[i].cpg_cycle;
		//RGニューロンの膜電位の保存
		sam->RG_F_V[i][k] = Unit[i].Neuron_V[RG_F];
		sam->RG_E_V[i][k] = Unit[i].Neuron_V[RG_E];
		//PFニューロンの膜電位の保存
		sam->PF_Sw_V[i][k] = Unit[i].Neuron_V[PF_Sw];
		sam->PF_St_V[i][k] = Unit[i].Neuron_V[PF_St];
		sam->PF_Lo_V[i][k] = Unit[i].Neuron_V[PF_Lo];
		sam->PF_Td_V[i][k] = Unit[i].Neuron_V[PF_Td];
		// 運動ニューロンの膜電位の保存
		sam->Mn_1_V[i][k] = Unit[i].Neuron_V[Mn_1];
		sam->Mn_2_V[i][k] = Unit[i].Neuron_V[Mn_2];
		sam->Mn_3_V[i][k] = Unit[i].Neuron_V[Mn_3];
		sam->Mn_4_V[i][k] = Unit[i].Neuron_V[Mn_4];
		sam->Mn_5_V[i][k] = Unit[i].Neuron_V[Mn_5];
		sam->Mn_6_V[i][k] = Unit[i].Neuron_V[Mn_6];
        //脚負荷情報の保存 シグモイド後
//		sam->feed_prs[i][k] = 1.0 / ( 1.0 + exp( -(load->load_value[i] - 3.0) * sigmoid_alpha) );
		//脚負荷情報の保存 シグモイド前
		sam->feed_prs[i][k] =load->load_value[i];
		//RG-Fの発火期間の保存
		sam->RG_F_period[i][k] = Unit[i].RG_F_period;
		//RG-Eの発火期間の保存
		sam->RG_E_period[i][k] = Unit[i].RG_E_period;
		//腰関節角度の保存
		sam->angle_hip[i][k] = enc->angle_hip_deg[i];
		//膝関節角度の保存
		sam->angle_knee[i][k] = enc->angle_knee_deg[i];
		//足首関節角度の保存
		sam->angle_ankle[i][k] = enc->angle_ankle_deg[i];
		//FF電圧値の保存
		sam->FF_data[i][k] = ad->FF_data[i];
		//接地情報(FF依存)
		sam->FF_touch[i][k] = load->FF_touch[i];
		//ポテンショメータ電圧値の保存
		sam->POT_data[i][k] = ad->POT_data[i];
		//足底プレート角度の保存
		sam->foot_angle[i][k] = ad->foot_angle[i];
		//足底プレート角速度の保存
		sam->foot_angle_vel[i][k] = ad->foot_angle_vel[i];
		//接地情報(ポテンショメータ依存)
		sam->POT_touch[i][k] = load->POT_touch[i];
		//接地情報(決定版)の保存
		sam->touch[i][k] = load->touch[i];
		//20191126追加　脚負荷用介在ニューロンIn_Feedの保存
		sam->In_Feed_V[i][k] = Unit[i].Neuron_V[In_Feed];
        //筋肉動作モードの保存
        sam->Valve_mode1[i][k] = valve_mode[i][0];
        sam->Valve_mode2[i][k] = valve_mode[i][1];
        sam->Valve_mode3[i][k] = valve_mode[i][2];
        sam->Valve_mode4[i][k] = valve_mode[i][3];
        sam->Valve_mode5[i][k] = valve_mode[i][4];
        sam->Valve_mode6[i][k] = valve_mode[i][5];
	}

	for(i=0; i<num_AXIS; i++) {
		// ジャイロセンサ出力[V]
		sam->gyro_data[i][k] = ad->gyro_data[i];
		// ジャイロセンサにより検出した胴体傾き[deg]
		sam->gyro_angle_deg[i][k] = ad->gyro_angle_deg[i] + ad->tilt_angle_offset_deg[i];
		// ジャイロセンサにより検出した胴体傾き[deg](ローパス通過後)
		sam->gyro_angle_lowp_deg[i][k] = ad->gyro_angle_lowp_deg[i] + ad->tilt_angle_offset_deg[i];
		// 加速度センサ出力[V]
		sam->acc_data[i][k] = ad->acc_data[i];
		// 加速度センサにより検出した胴体傾き[deg]
		sam->acc_angle_deg[i][k] = ad->acc_angle_deg[i];
		// 加速度センサにより検出した胴体傾き[deg](ローパス通過後)
		sam->acc_angle_lowp_deg[i][k] = ad->acc_angle_lowp_deg[i];
		// ジャイロセンサと加速度センサを併用してドリフト補正した胴体傾き角度[deg]
		sam->tilt_angle_deg[i][k] = ad->tilt_angle_deg[i] + ad->tilt_angle_offset_deg[i];
	}

//printf(" %f", sam->gyro_angle_deg[pitch][k]);
//printf(" %f\n", ad->gyro_angle_deg[pitch]);

	//トレッドミル速度の保存
	sam->speed_treadmill[k] = enc->speed_treadmill;

	//人工筋肉内気圧の取得
	sam->atmosphere[k] = ad->atmosphere;

    //電磁弁のONOFFを取得
    sam->muscle_io[k] = muscle_io;

}


/*----------------------------------------------*/
/*	ファイルからパラメータを読み込む	*/
/*	file_output()				*/
/*----------------------------------------------*/
int file_input(float buf[30][30], float c_f[][DEF_B], float a_f[][DEF_B], float b_f[][DEF_B], float w_f[][DEF_B], float c_h[][DEF_B], float a_h[][DEF_B], float b_h[][DEF_B], float w_h[][DEF_B])
{
	//------変数宣言------//
	int i, j;	// カウンタ
	FILE	*fpin_param = fopen("0_param.txt", "r");		//チューニング用入力ファイルを開く

	//ファイル展開時の安全装置
	if (fpin_param == NULL) {
		printf("ファイルが使用中です．\n");
		return 1;
	}

	// MAT内のパラメータ(PF->Mn以外)入力 -------------------------------------------------------------------------------------
	//	PF->Mn以外の結合荷重は4脚で同じ
	//20191126 In_Feed用にbuf[i][24]追加
	// MATc_fore/hind
	for(i=0; i<MAT_C_INPUTS; i++){
		fscanf(fpin_param, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
					&buf[i][0], &buf[i][1], &buf[i][2], &buf[i][3], &buf[i][4], &buf[i][5],
					&buf[i][6], &buf[i][7], &buf[i][8], &buf[i][9], &buf[i][10], &buf[i][11],
					&buf[i][12], &buf[i][13], &buf[i][14], &buf[i][15], &buf[i][16], &buf[i][17],
					&buf[i][18], &buf[i][19], &buf[i][20], &buf[i][21], &buf[i][22], &buf[i][23], &buf[i][24]	);
		for(j=0; j<DEF_B; j++)
			c_f[i][j] = c_h[i][j] = buf[i][j];
	}
	// MATa_fore/hind
	for(i=0; i<MAT_A_INPUTS; i++){
		fscanf(fpin_param, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
					&buf[i][0], &buf[i][1], &buf[i][2], &buf[i][3], &buf[i][4], &buf[i][5],
					&buf[i][6], &buf[i][7], &buf[i][8], &buf[i][9], &buf[i][10], &buf[i][11],
					&buf[i][12], &buf[i][13], &buf[i][14], &buf[i][15], &buf[i][16], &buf[i][17],
					&buf[i][18], &buf[i][19], &buf[i][20], &buf[i][21], &buf[i][22], &buf[i][23], &buf[i][24]	);
		for(j=0; j<DEF_B; j++)
			a_f[i][j] = a_h[i][j] = buf[i][j];
	}
	// MATb_fore/hind
	for(i=0; i<MAT_B_INPUTS; i++){
		fscanf(fpin_param, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
					&buf[i][0], &buf[i][1], &buf[i][2], &buf[i][3], &buf[i][4], &buf[i][5],
					&buf[i][6], &buf[i][7], &buf[i][8], &buf[i][9], &buf[i][10], &buf[i][11],
					&buf[i][12], &buf[i][13], &buf[i][14], &buf[i][15], &buf[i][16], &buf[i][17],
					&buf[i][18], &buf[i][19], &buf[i][20], &buf[i][21], &buf[i][22], &buf[i][23], &buf[i][24]	);
		for(j=0; j<DEF_B; j++)
			b_f[i][j] = b_h[i][j] = buf[i][j];
	}
	// MATw_fore/hind
	for(i=0; i<MAT_W_INPUTS; i++){
		fscanf(fpin_param, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
					&buf[i][0], &buf[i][1], &buf[i][2], &buf[i][3], &buf[i][4], &buf[i][5],
					&buf[i][6], &buf[i][7], &buf[i][8], &buf[i][9], &buf[i][10], &buf[i][11],
					&buf[i][12], &buf[i][13], &buf[i][14], &buf[i][15], &buf[i][16], &buf[i][17],
					&buf[i][18], &buf[i][19], &buf[i][20], &buf[i][21], &buf[i][22], &buf[i][23], &buf[i][24]	);
		for(j=0; j<DEF_B; j++)
			w_f[i][j] = w_h[i][j] = buf[i][j];
	}
	// -----------------------------------------------------------------------------------------------------------------------
	// PF->Mnの結合荷重入力 --------------------------------------------------------------------------------------------------
	//	PF->Mnの結合荷重は前後脚で異なる
	for(i=0; i<8; i++){
		fscanf(fpin_param, "%f %f %f %f %f %f", &buf[i][0], &buf[i][1], &buf[i][2], &buf[i][3], &buf[i][4], &buf[i][5]);
	}
	for(j=0; j<MUSCLES_PER_LEG; j++)	{
		// 前脚
		a_f[2][Mn_1+j] = buf[0][j];
		a_f[3][Mn_1+j] = buf[1][j];
		a_f[5][Mn_1+j] = buf[2][j];
		a_f[6][Mn_1+j] = buf[3][j];
		// 後脚
		a_h[2][Mn_1+j] = buf[4][j];
		a_h[3][Mn_1+j] = buf[5][j];
		a_h[5][Mn_1+j] = buf[6][j];
		a_h[6][Mn_1+j] = buf[7][j];
	}
	// -----------------------------------------------------------------------------------------------------------------------

	// 読み込みファイルを 閉じる
	fclose(fpin_param);
	return 0;
}

/*----------------------------------------------*/
/*	ファイルへサンプルデータを書き出す	*/
/*	file_output()				*/
/*----------------------------------------------*/
int file_output(int k, SAMType *sam)
{
	/*------変数宣言------*/
	int	i, sam_n;	//カウンタ
	float	cpg_phase[UNITS];
	float	RG_F_fV[UNITS];
	float	RG_E_fV[UNITS];
	float	PF_Sw_fV[UNITS];
	float	PF_St_fV[UNITS];
	float	PF_Lo_fV[UNITS];
	float	PF_Td_fV[UNITS];
	float	Mn_1_fV[UNITS];
	float	Mn_2_fV[UNITS];
	float	Mn_3_fV[UNITS];
	float	Mn_4_fV[UNITS];
	float	Mn_5_fV[UNITS];
	float	Mn_6_fV[UNITS];
    int	Mn_1_ONOFF[UNITS];
	int	Mn_2_ONOFF[UNITS];
	int	Mn_3_ONOFF[UNITS];
	int	Mn_4_ONOFF[UNITS];
	int	Mn_5_ONOFF[UNITS];
	int	Mn_6_ONOFF[UNITS];
	float	Muscle6_len[UNITS];
	float	duty_ratio[UNITS];
	float In_Feed_fV[UNITS];

	//取得データ保存ファイル
	FILE	*fp_neuron = fopen("0_log_neuron.txt", "w");	//CPG内の各ニューロンの発火率を保存
	FILE	*fp_sensor = fopen("0_log_sensor.txt", "w");	//脚の関節角度や胴体傾き（予定）などを保存


	//ファイル展開時の安全装置
	if (fp_neuron == NULL) {
		printf("ファイルが使用中です．neuron\n");
		return 1;
	}
	if (fp_sensor == NULL) {
		printf("ファイルが使用中です．sensor\n");
		return 1;
	}

	/*------ニューロンデータ書き出し------*/
	//保存するデータ項目の列挙
	fprintf(fp_neuron, "t \t vel_treadmill \t ");	//時刻、トレッドミル速度
	fprintf(fp_neuron, "CPG_LF \t CPG_LH \t CPG_RF \t CPG_RH \t ");	//CPG波形
	fprintf(fp_neuron, "Cycle_LF \t Cycle_LH \t Cycle_RF \t Cycle_RH \t ");	// CPG周期

	fprintf(fp_neuron, "RG-F_fV_LF \t RG-E_fV_LF \t ");	//RGニューロンの発火率（左前）
	fprintf(fp_neuron, "RG-F_fV_LH \t RG-E_fV_LH \t ");	//RGニューロンの発火率（左後）
	fprintf(fp_neuron, "RG-F_fV_RF \t RG-E_fV_RF \t ");	//RGニューロンの発火率（右前）
	fprintf(fp_neuron, "RG-F_fV_RH \t RG-E_fV_RH \t ");	//RGニューロンの発火率（右後）

	fprintf(fp_neuron, "RG-F_V_LF \t RG-E_V_LF \t ");	//RGニューロンの膜電位（左前）
	fprintf(fp_neuron, "RG-F_V_LH \t RG-E_V_LH \t ");	//RGニューロンの膜電位（左後）
	fprintf(fp_neuron, "RG-F_V_RF \t RG-E_V_RF \t ");	//RGニューロンの膜電位（右前）
	fprintf(fp_neuron, "RG-F_V_RH \t RG-E_V_RH \t ");	//RGニューロンの膜電位（右後）

	fprintf(fp_neuron, "PF-Sw_LF \t PF-St_LF \t PF-Lo_LF \t PF-Td_LF \t ");	//PFニューロンの発火率（左前）
	fprintf(fp_neuron, "PF-Sw_LH \t PF-St_LH \t PF-Lo_LH \t PF-Td_LH \t ");	//PFニューロンの発火率（左後）
	fprintf(fp_neuron, "PF-Sw_RF \t PF-St_RF \t PF-Lo_RF \t PF-Td_RF \t ");	//PFニューロンの発火率（右前）
	fprintf(fp_neuron, "PF-Sw_RH \t PF-St_RH \t PF-Lo_RH \t PF-Td_RH \t ");	//PFニューロンの発火率（右後）

	fprintf(fp_neuron, "Mn-1_LF \t Mn-2_LF \t Mn-3_LF \t Mn-4_LF \t Mn-5_LF \t Mn-6_LF \t ");	//運動ニューロンの発火率（左前）
	fprintf(fp_neuron, "Mn-1_LH \t Mn-2_LH \t Mn-3_LH \t Mn-4_LH \t Mn-5_LH \t Mn-6_LH \t ");	//運動ニューロンの発火率（左後）
	fprintf(fp_neuron, "Mn-1_RF \t Mn-2_RF \t Mn-3_RF \t Mn-4_RF \t Mn-5_RF \t Mn-6_RF \t ");	//運動ニューロンの発火率（右前）
	fprintf(fp_neuron, "Mn-1_RH \t Mn-2_RH \t Mn-3_RH \t Mn-4_RH \t Mn-5_RH \t Mn-6_RH \t ");	//運動ニューロンの発火率（右後）

	fprintf(fp_neuron, "Mn-1_LF_ONOFF \t Mn-2_LF_ONOFF \t Mn-3_LF_ONOFF \t Mn-4_LF_ONOFF \t Mn-5_LF_ONOFF \t Mn-6_LF_ONOFF \t ");	//運動ニューロンのONOFF（左前）
	fprintf(fp_neuron, "Mn-1_LH_ONOFF \t Mn-2_LH_ONOFF \t Mn-3_LH_ONOFF \t Mn-4_LH_ONOFF \t Mn-5_LH_ONOFF \t Mn-6_LH_ONOFF \t ");	//運動ニューロンのONOFF（左後）
	fprintf(fp_neuron, "Mn-1_RF_ONOFF \t Mn-2_RF_ONOFF \t Mn-3_RF_ONOFF \t Mn-4_RF_ONOFF \t Mn-5_RF_ONOFF \t Mn-6_RF_ONOFF \t ");	//運動ニューロンのONOFF（右前）
	fprintf(fp_neuron, "Mn-1_RH_ONOFF \t Mn-2_RH_ONOFF \t Mn-3_RH_ONOFF \t Mn-4_RH_ONOFF \t Mn-5_RH_ONOFF \t Mn-6_RH_ONOFF \t ");	//運動ニューロンのONOFF（右後）

	fprintf(fp_neuron, "touch_LF \t touch_LH \t touch_RF \t touch_RH \t ");	//接地情報

	fprintf(fp_neuron, "feed_prs_LF \t feed_prs_LH \t feed_prs_RF \t feed_prs_RH \t ");	//脚負荷情報

	fprintf(fp_neuron, "RG-F_period_LF \t RG-E_period_LF \t RG-F_period_LH \t RG-E_period_LH \t RG-F_period_RF \t RG-E_period_RF \t RG-F_period_RH \t RG-E_period_RH \t ");	// RGの発火期間

	fprintf(fp_neuron, "duty_ratio_LF \t duty_ratio_LH \t duty_ratio_RF \t duty_ratio_RH \t "); //duty比

	fprintf(fp_neuron, "In_Feed_LF \t In_Feed_LH \t In_Feed_RF \t In_Feed_RH \t "); //20191126　脚負荷を通す介在ニューロンの発火率

	//fprintf(fp_neuron, "In_F_LF \t In_E_LF \t In_LF \t Inab_E_LF \t In_PF-Sw_LF \t In_PF-St_LF \t In_PF-Lo_LF \t In_PF-Td_LF \t");	//介在ニューロン（左前）

    fprintf(fp_neuron,"muscle_io \t ");

	fprintf(fp_neuron, "\n");	// 改行

	//保存データの書き出し
	for(sam_n=0; sam_n<k; sam_n++){
		fprintf(fp_neuron, "%f \t %f \t ", sam->t[sam_n], sam->speed_treadmill[sam_n]);

		// CPG波形・周期
		for (i=0; i<UNITS; i++)
			cpg_phase[i] = GetfV(sam->RG_E_V[i][sam_n], RG_E) - GetfV(sam->RG_F_V[i][sam_n], RG_F);
		fprintf(fp_neuron, "%f \t %f \t %f \t %f \t ", cpg_phase[0], cpg_phase[1], cpg_phase[2], cpg_phase[3]);
		fprintf(fp_neuron, "%f \t %f \t %f \t %f \t ", sam->cpg_cycle[0][sam_n], sam->cpg_cycle[1][sam_n], sam->cpg_cycle[2][sam_n], sam->cpg_cycle[3][sam_n]);

		//RGニューロンの発火率
		for(i=0; i<UNITS; i++) {
			RG_F_fV[i] = GetfV(sam->RG_F_V[i][sam_n], RG_F);
			RG_E_fV[i] = GetfV(sam->RG_E_V[i][sam_n], RG_E);
			fprintf(fp_neuron, "%f \t %f \t ", RG_F_fV[i], RG_E_fV[i]);
		}
		//RGニューロンの膜電位
		for(i=0; i<UNITS; i++)
			fprintf(fp_neuron, "%f \t %f \t ", sam->RG_F_V[i][sam_n], sam->RG_E_V[i][sam_n]);

		//PFニューロンの発火率
		for(i=0; i<UNITS; i++) {
			PF_Sw_fV[i] = GetfV(sam->PF_Sw_V[i][sam_n], PF_Sw);
			PF_St_fV[i] = GetfV(sam->PF_St_V[i][sam_n], PF_St);
			PF_Lo_fV[i] = GetfV(sam->PF_Lo_V[i][sam_n], PF_Lo);
			PF_Td_fV[i] = GetfV(sam->PF_Td_V[i][sam_n], PF_Td);
			fprintf(fp_neuron, "%f \t %f \t %f \t %f \t ", PF_Sw_fV[i], PF_St_fV[i], PF_Lo_fV[i], PF_Td_fV[i]);
		}
		//Mnニューロンの発火率
		for(i=0; i<UNITS; i++) {
			Mn_1_fV[i] = GetfV(sam->Mn_1_V[i][sam_n], Mn_1);
			Mn_2_fV[i] = GetfV(sam->Mn_2_V[i][sam_n], Mn_2);
			Mn_3_fV[i] = GetfV(sam->Mn_3_V[i][sam_n], Mn_3);
			Mn_4_fV[i] = GetfV(sam->Mn_4_V[i][sam_n], Mn_4);
			Mn_5_fV[i] = GetfV(sam->Mn_5_V[i][sam_n], Mn_5);
			Mn_6_fV[i] = GetfV(sam->Mn_6_V[i][sam_n], Mn_6);
			fprintf(fp_neuron, "%f \t %f \t %f \t %f \t %f \t %f \t ", Mn_1_fV[i], Mn_2_fV[i], Mn_3_fV[i], Mn_4_fV[i], Mn_5_fV[i], Mn_6_fV[i]);
		}

		//MnニューロンのONOFF
		for(i=0; i<UNITS; i++) {
			Mn_1_ONOFF[i] = sam->Valve_mode1[i][sam_n];
            Mn_2_ONOFF[i] = sam->Valve_mode2[i][sam_n];
            Mn_3_ONOFF[i] = sam->Valve_mode3[i][sam_n];
            Mn_4_ONOFF[i] = sam->Valve_mode4[i][sam_n];
            Mn_5_ONOFF[i] = sam->Valve_mode5[i][sam_n];
            Mn_6_ONOFF[i] = sam->Valve_mode6[i][sam_n];

            fprintf(fp_neuron, "%d \t %d \t %d \t %d \t %d \t %d \t ", Mn_1_ONOFF[i], Mn_2_ONOFF[i], Mn_3_ONOFF[i], Mn_4_ONOFF[i], Mn_5_ONOFF[i], Mn_6_ONOFF[i]);

		}

		//接地情報
		fprintf(fp_neuron, "%d \t %d \t %d \t %d \t ", sam->touch[0][sam_n], sam->touch[1][sam_n], sam->touch[2][sam_n], sam->touch[3][sam_n]);

		//脚負荷情報
		for(i=0; i<UNITS; i++)
			fprintf(fp_neuron, "%f \t ", sam->feed_prs[i][sam_n]);

		//RGの発火期間
		for(i=0; i<UNITS; i++)
			fprintf(fp_neuron, "%f \t %f \t ", sam->RG_F_period[i][sam_n], sam->RG_E_period[i][sam_n]);

		//duty比
		for(i=0; i<UNITS; i++) {
			duty_ratio[i] = sam->RG_E_period[i][sam_n] / (sam->RG_F_period[i][sam_n] + sam->RG_E_period[i][sam_n]);
			fprintf(fp_neuron, "%f \t ", duty_ratio[i]);
		}

		for(i=0; i<UNITS; i++) {
				In_Feed_fV[i] = GetfV(sam->In_Feed_V[i][sam_n], In_Feed);
				fprintf(fp_neuron, "%f \t ", In_Feed_fV[i]);
		}

        fprintf(fp_neuron, "%d \t ", sam->muscle_io[sam_n]);

		fprintf(fp_neuron, "\n");	// 改行
	}

	/*------センサデータの書き出し------*/
	//保存するデータ項目の列挙
	fprintf(fp_sensor, "t \t vel_treadmill \t");	//時刻，トレッドミル速度

	fprintf(fp_sensor, "CPG_LF \t CPG_LH \t CPG_RF \t CPG_RH \t");		// CPG波形
	fprintf(fp_sensor, "Cycle_LF \t Cycle_LH \t Cycle_RF \t Cycle_RH \t");	// CPG周期

	fprintf(fp_sensor, "scapula_LF \t shoulder_LF \t elbow_LF \t ");	// 左前脚
	fprintf(fp_sensor, "hip_LH \t knee_LH \t ankle_LH \t ");		// 左後脚
	fprintf(fp_sensor, "scapula_RF \t shoulder_RF \t elbow_RF \t ");	// 右前脚
	fprintf(fp_sensor, "hip_RH \t knee_RH \t ankle_RH \t ");		// 右後脚


	fprintf(fp_sensor, "touch_LF \t touch_LH \t touch_RF \t touch_RH \t ");	// 接地情報

	fprintf(fp_sensor, "Muscle6_len_LF \t Muscle6_len_LH \t Muscle6_len_RF \t Muscle6_len_RH \t ");	// 6番筋長さ

	fprintf(fp_sensor, "tilt_angle_pitch \t tilt_angle_roll \t ");			// 胴体傾き角度(決定版)

	fprintf(fp_sensor, "FF_data_LF \t FF_data_LH \t FF_data_RF \t FF_data_RH \t ");		// FF電圧値
	fprintf(fp_sensor, "FF_touch_LF \t FF_touch_LH \t FF_touch_RF \t FF_touch_RH \t ");	// 接地情報(FF依存)

	fprintf(fp_sensor, "POT_data_LF \t POT_data_LH \t POT_data_RF \t POT_data_RH \t "); 				// ポテンショメータ電圧値
	fprintf(fp_sensor, "foot_angle_LF \t foot_angle_LH \t foot_angle_RF \t foot_angle_RH \t ");			// 足底プレート角度
	fprintf(fp_sensor, "foot_angle_vel_LF \t foot_angle_vel_LH \t foot_angle_vel_RF \t foot_angle_vel_RH \t ");	// 足底プレート角速度
	fprintf(fp_sensor, "POT_touch_LF \t POT_touch_LH \t POT_touch_RF \t POT_touch_RH \t "); 			// 接地情報(ポテンショメータ依存)

	fprintf(fp_sensor, "gyro_data_pitch \t gyro_data_roll \t ");			// ジャイロセンサ出力
	fprintf(fp_sensor, "gyro_angle_pitch \t gyro_angle_roll \t ");			// 胴体傾き角度(ジャイロセンサ)
	fprintf(fp_sensor, "gyro_angle_lowp_pitch \t gyro_angle_lowp_roll \t ");	// ローパス通した後の胴体傾き角度(ジャイロセンサ)

	fprintf(fp_sensor, "acc_data_pitch \t acc_data_roll \t ");			// 加速度センサ出力
	fprintf(fp_sensor, "acc_angle_pitch \t acc_angle_roll \t ");			// 胴体傾き角度(加速度センサ)
	fprintf(fp_sensor, "acc_angle_lowp_pitch \t acc_angle_lowp_roll \t ");		// ローパス通した後の胴体傾き角度(加速度センサ)

	fprintf(fp_sensor, "atmosphere \t ");	// 人工筋肉内気圧

	fprintf(fp_sensor, "\n");	// 改行

	//保存データの書き出し
	for(sam_n=0; sam_n<k; sam_n++){
		fprintf(fp_sensor, "%f \t %f \t ", sam->t[sam_n], sam->speed_treadmill[sam_n]);

		// CPG波形・周期
		for (i=0; i<UNITS; i++)
			cpg_phase[i] = GetfV(sam->RG_E_V[i][sam_n], RG_E) - GetfV(sam->RG_F_V[i][sam_n], RG_F);
		fprintf(fp_sensor, "%f \t %f \t %f \t %f \t ", cpg_phase[0], cpg_phase[1], cpg_phase[2], cpg_phase[3]);
		fprintf(fp_sensor, "%f \t %f \t %f \t %f \t ", sam->cpg_cycle[0][sam_n], sam->cpg_cycle[1][sam_n], sam->cpg_cycle[2][sam_n], sam->cpg_cycle[3][sam_n]);

		//関節データ
		for(i=0; i<UNITS; i++)
			fprintf(fp_sensor, "%f \t %f \t %f \t ", sam->angle_hip[i][sam_n], sam->angle_knee[i][sam_n], sam->angle_ankle[i][sam_n]);


		//接地情報
		fprintf(fp_sensor, "%d \t %d \t %d \t %d \t ", sam->touch[0][sam_n], sam->touch[1][sam_n], sam->touch[2][sam_n], sam->touch[3][sam_n]);

		// 6番筋長さ
		for (i=0; i<UNITS; i++) {
			if (i==LF || i==RF)	// 前脚
				Muscle6_len[i] = Calc_Muscle6_F_len(sam->angle_knee[i][sam_n], sam->angle_ankle[i][sam_n]);
			else	// 後脚
				Muscle6_len[i] = Calc_Muscle6_H_len(sam->angle_knee[i][sam_n], sam->angle_ankle[i][sam_n]);
		}
		fprintf(fp_sensor, "%f \t %f \t %f \t %f \t ", Muscle6_len[0], Muscle6_len[1], Muscle6_len[2], Muscle6_len[3]);

		//胴体傾き角度(決定版)
		fprintf(fp_sensor, "%f \t %f \t ", sam->tilt_angle_deg[0][sam_n], sam->tilt_angle_deg[1][sam_n]);

		//FF電圧値
		fprintf(fp_sensor, "%f \t %f \t %f \t %f \t ", sam->FF_data[0][sam_n], sam->FF_data[1][sam_n], sam->FF_data[2][sam_n], sam->FF_data[3][sam_n]);
		//接地情報(FF依存)
		fprintf(fp_sensor, "%d \t %d \t %d \t %d \t ", sam->FF_touch[0][sam_n], sam->FF_touch[1][sam_n], sam->FF_touch[2][sam_n], sam->FF_touch[3][sam_n]);

		//ポテンショメータ電圧値
		fprintf(fp_sensor, "%f \t %f \t %f \t %f \t ", sam->POT_data[0][sam_n], sam->POT_data[1][sam_n], sam->POT_data[2][sam_n], sam->POT_data[3][sam_n]);
		//足底プレート角度
		fprintf(fp_sensor, "%f \t %f \t %f \t %f \t ", sam->foot_angle[0][sam_n], sam->foot_angle[1][sam_n], sam->foot_angle[2][sam_n], sam->foot_angle[3][sam_n]);
		//足底プレート角速度
		fprintf(fp_sensor, "%f \t %f \t %f \t %f \t ", sam->foot_angle_vel[0][sam_n], sam->foot_angle_vel[1][sam_n], sam->foot_angle_vel[2][sam_n], sam->foot_angle_vel[3][sam_n]);
		//接地情報(ポテンショメータ依存)
		fprintf(fp_sensor, "%d \t %d \t %d \t %d \t ", sam->POT_touch[0][sam_n], sam->POT_touch[1][sam_n], sam->POT_touch[2][sam_n], sam->POT_touch[3][sam_n]);

		//ジャイロセンサ出力
		fprintf(fp_sensor, "%f \t %f \t ", sam->gyro_data[0][sam_n], sam->gyro_data[1][sam_n]);
		//胴体傾き角度(ジャイロセンサ)
		fprintf(fp_sensor, "%f \t %f \t ", sam->gyro_angle_deg[0][sam_n], sam->gyro_angle_deg[1][sam_n]);
		//ローパス通した後の胴体傾き角度(ジャイロセンサ)
		fprintf(fp_sensor, "%f \t %f \t ", sam->gyro_angle_lowp_deg[0][sam_n], sam->gyro_angle_lowp_deg[1][sam_n]);
		//加速度センサ出力
		fprintf(fp_sensor, "%f \t %f \t ", sam->acc_data[0][sam_n], sam->acc_data[1][sam_n]);
		//胴体傾き角度(加速度センサ)
		fprintf(fp_sensor, "%f \t %f \t ", sam->acc_angle_deg[0][sam_n], sam->acc_angle_deg[1][sam_n]);
		//ローパス通した後の胴体傾き角度(加速度センサ)
		fprintf(fp_sensor, "%f \t %f \t ", sam->acc_angle_lowp_deg[0][sam_n], sam->acc_angle_lowp_deg[1][sam_n]);

		//人工筋肉内気圧
		fprintf(fp_sensor, "%f \t ", sam->atmosphere[sam_n]);

		fprintf(fp_sensor, "\n");	// 改行
	}

	//書き込みファイルを 閉じる
	fclose(fp_neuron);
	fclose(fp_sensor);
	return 0;
}

// 時間経過によるパラメータ設定
void param(float d[2], LOADType *load, float t)
{
	int i;	// カウンタ

	// 上位入力の変更
	if (t<Time_State1){	// 初期パラメータ
		d[0] = Sup_Driven1;
		d[1] = Ext_Driven1;
	}
	else if (t<Time_State2){	// パラメータ変更中
		d[0] = Sup_Driven1 + (Sup_Driven2 - Sup_Driven1)/(Time_State2 - Time_State1) * (t - Time_State1);
		d[1] = Ext_Driven1 + (Ext_Driven2 - Ext_Driven1)/(Time_State2 - Time_State1) * (t - Time_State1);
	}
	else{	// パラメータ変更後
		d[0] = Sup_Driven2;
		d[1] = Ext_Driven2;
	}

	// 人工筋肉の張力特性の変更
	for(i=0; i<UNITS; i++) {
		if(i == LF || i == RF) { // 前脚
			if (t<Time_State1)	// 初期張力特性
				load->load_char[i] = Load_char1_F;
			else if (t<Time_State2)	// 張力特性変更中
				load->load_char[i] = Load_char1_F + (Load_char2_F - Load_char1_F)/(Time_State2 - Time_State1) * (t - Time_State1);
			else	// 張力特性変更後
				load->load_char[i] = Load_char2_F;
		}
		else { // 後脚
			if (t<Time_State1)	// 初期張力特性
				load->load_char[i] = Load_char1_H;
			else if (t<Time_State2)	// 張力特性変更中
				load->load_char[i] = Load_char1_H + (Load_char2_H - Load_char1_H)/(Time_State2 - Time_State1) * (t - Time_State1);
			else	// 張力特性変更後
				load->load_char[i] = Load_char2_H;
		}
	}
}


// 関数の平均処理時間と最大処理時間を表示
void Calc_proc_time(int k, float t_p_sum, float t_p_max)
{
	float	t_p_ave;	// 関数の平均処理時間

	// 平均処理時間の計算(関数の合計処理時間 / 関数の実行回数)
	t_p_ave = t_p_sum / (float)k;

	printf("Specified Function's ...\n");
	printf("\t Mean Processing Time : %f[ms]\n", t_p_ave * 1000.0);
	printf("\t Max Processing Time : %f[ms]\n\n", t_p_max * 1000.0);


	/* この関数を使用する場合の注意 --------------------------

	処理時間を計測したい関数を"function_a()"とした場合，

	// -------------------------------------------------------------------------------------------- //
	//	t1 = (float)rdtsc();				// 関数処理直前の時刻			//
	//	function_a();					// 関数を実行				//
	//	t2 = (float)rdtsc();				// 関数処理直後の時刻			//
	//	t_proc = (float)((t2-t1)/tmu_frq/1.0e+6);	// 関数の処理にかかった時間を計算	//
	//	if(t_proc > t_proc_max) t_proc_max = t_proc;	// 最大処理時間の更新			//
	//	k_func++;					// 関数実行回数更新			//
	// -------------------------------------------------------------------------------------------- //

	上記のような記述を"1_main.c"内に追加する必要がある

	------------------------------------------------------- */
}
