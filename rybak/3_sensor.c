// -------------------------------------------- //
// 	センサ情報取得に関する関数群		//
//		sensor.c			//
// -------------------------------------------- //

#include "8_define.h"
#include "9_user_param.h"

#define ENC_RESOLUTION	1024	// エンコーダパルス数
#define	MP	2.0		// 逓倍
#define	LP_Tr	50.0		// トレッドミル速度補正用定数
#define	LP_FF	10.0		// FlexiForce補正用定数

#define GYRO_SENS		(0.67)	// ジャイロセンサの感度[mV/deg/sec]
#define GYRO_AMP		(3.0)	// ジャイロセンサの増幅度(5.0/0.88?)
#define GYRO_SENS_AMP		(GYRO_SENS * GYRO_AMP)	//ジャイロセンサの増幅後の感度[mV/deg/sec]

#define ACC_SENS		(1.0)	// 加速度センサの感度[V/G](1G = 9.8m/s^2)
#define ACC_AMP			(1.0)	// 加速度センサの増幅度


int ENC_HIP_CH[UNITS] = {0, 4, 8, 12};		// ENCチャンネル番号(腰/肩胛骨)
int ENC_KNEE_CH[UNITS] = {1, 5, 9, 13};	// ENCチャンネル番号(膝/肩)
int ENC_ANKLE_CH[UNITS] = {2, 6, 10, 14}; 	// ENCチャンネル番号(足首/肘)
int ENC_TREADMILL_CH = 15;			// ENCチャンネル番号(トレッドミル)

int AD_FF_CH[UNITS] = {1, 4, 8, 12};		// ADチャンネル番号(FlexiForce)
int AD_POT_CH[UNITS] = {0, 5, 9, 13};		// ADチャンネル番号(ポテンショメータ)
int AD_GYRO_CH[num_AXIS] = {2, 14};		// ADチャンネル番号(ジャイロセンサ)
int AD_ACC_CH[num_AXIS] = {10, 6};		// ADチャンネル番号(加速度センサ)
int AD_AIR_PRESS_CH = 15;			// ADチャンネル番号(空気圧センサ)

/*
//------------------------------//
//	各関節角度の取得	//
//	angle_get()		//
//------------------------------//
void angle_get(int i, ENCType *enc)
{
	//エンコーダ値を取得
	getEnc(ENC_HIP_CH[i], &(enc->data_hip[i]));		//腰(肩甲骨)関節
	getEnc(ENC_KNEE_CH[i], &(enc->data_knee[i]));		//膝(肩)関節
	getEnc(ENC_ANKLE_CH[i], &(enc->data_ankle[i]));		//足首(肘)関節

	//ひとつ前の角度[rad]を更新
	enc->angle_hip_rad_last[i] = enc->angle_hip_rad[i];	//腰(肩甲骨)関節
	enc->angle_knee_rad_last[i] = enc->angle_knee_rad[i];	//膝(肩)関節
	enc->angle_ankle_rad_last[i] = enc->angle_ankle_rad[i];	//足首(肘)関節

	//エンコーダ値を角度[rad]へ変換
	enc->angle_hip_rad[i] = (float)((2.0 * PI * enc->data_hip[i]) / (ENC_RESOLUTION * MP));		//腰(肩甲骨)関節
	enc->angle_knee_rad[i] = (float)((2.0 * PI * enc->data_knee[i]) / (ENC_RESOLUTION * MP));	//膝(肩)関節
	enc->angle_ankle_rad[i] = (float)((2.0 * PI * enc->data_ankle[i]) / (ENC_RESOLUTION * MP));	//足首(肘)関節

	//回転方向一致(右側脚はエンコーダが逆に取り付けられているため)
	if(i == RF || i == RH){
		enc->angle_hip_rad[i] *= -1.0;		//腰(肩甲骨)関節
		enc->angle_knee_rad[i] *= -1.0;		//膝(肩)関節
		enc->angle_ankle_rad[i] *= -1.0;	//足首(肘)関節
	}

	//角度[deg]
	enc->angle_hip_deg[i] = enc->angle_hip_rad[i] * radtodeg;	//腰(肩甲骨)関節
	enc->angle_knee_deg[i] = enc->angle_knee_rad[i] * radtodeg;	//膝(肩)関節
	enc->angle_ankle_deg[i] = enc->angle_ankle_rad[i] * radtodeg;	//足首(肘)関節
}
*/

// ---------------------------- //
// 	4脚分の各関節角度取得	//
//	all_angle_get()		//
// ---------------------------- //
void all_angle_get(ENCType *enc)
{
	int i;
	int data_hip, data_knee, data_ankle;

	// 各関節 -------------------------------------------------------------------------------------------------
	//エンコーダ値を角度[deg]へ変換
	for(i=0; i<UNITS; i++) {	// 1 / (ENC_RESOLUTION * MP) ≒ 0.0004883
		// エンコーダ値を取得
		getEnc(ENC_HIP_CH[i], &data_hip);		//腰(肩甲骨)関節
		getEnc(ENC_KNEE_CH[i], &data_knee);		//膝(肩)関節
		getEnc(ENC_ANKLE_CH[i], &data_ankle);		//足首(肘)関節

		// 角度[deg]に変換（相対的なもの）
		data_hip = (float)((360.0 * data_hip) * 0.0004883);	//腰(肩甲骨)関節
		data_knee = (float)((360.0 * data_knee) * 0.0004883);	//膝(肩)関節
		data_ankle = (float)((360.0 * data_ankle) * 0.0004883);	//足首(肘)関節

		// 回転方向一致(右側脚はエンコーダが逆に取り付けられているため)
		if(i == LF || i == LH){
			data_hip *= -1.0;		//腰(肩甲骨)関節
			data_knee *= -1.0;		//膝(肩)関節
			data_ankle *= -1.0;	//足首(肘)関節
		}

		// 最終的な角度[deg]を取得（絶対的なもの）
		if (i == LF || i == RF) {	// 前脚
			enc->angle_hip_deg[i] = data_hip + ANGLE_SCAPULA_MIN;		// 肩甲骨関節
			enc->angle_knee_deg[i] = data_knee + ANGLE_SHOULDER_MIN;	// 肩関節
			enc->angle_ankle_deg[i] = data_ankle + ANGLE_ELBOW_MIN;		// 肘関節
		}
		else {		// 後脚
			enc->angle_hip_deg[i] = data_hip + ANGLE_HIP_MIN;		// 腰関節
			enc->angle_knee_deg[i] = data_knee + ANGLE_KNEE_MIN;		// 膝関節
			enc->angle_ankle_deg[i] = data_ankle + ANGLE_ANKLE_MIN;		// 足首関節
		}
	}
}


//--------------------------------------//
//	トレッドミル速度の取得		//
//	speed_treadmill_get()		//
//--------------------------------------//
void speed_treadmill_get(ENCType *enc, float h)
{
	int data_treadmill;
	float angle_treadmill;
	float speed_treadmill;

	// エンコーダ値を取得
	getEnc(ENC_TREADMILL_CH, &data_treadmill);
	// エンコーダ値を角度[rad]へ変換
	angle_treadmill = (float)((2.0 * PI * data_treadmill) * 0.0004883);	// 変換
	// トレッドミル速度v = Δθ * r / Δt
	speed_treadmill = (angle_treadmill - enc->angle_treadmill_rad) * 0.025 / h; //* 5.0e2;// 1.0/h = 500;
	// ローパスフィルタ
//	speed_treadmill = (enc->speed_treadmill * LP_Tr + speed_treadmill) / (LP_Tr + 1.0);
	speed_treadmill = (enc->speed_treadmill * 50.0 + speed_treadmill) * 0.019608;	// LP_Tr = 50.0
	// 更新
	enc->angle_treadmill_rad = angle_treadmill;
	enc->speed_treadmill = speed_treadmill;
}


/*
//------------------------------------------------------//
//	疑似脚負荷取得(6番筋長の変位から脚負荷推定)	//
//			load_get()			//
//------------------------------------------------------//
void load_get(int i, LOADType *load, RybakUnit *Unit)
{
	if(load->flag_load[i] == 1 && Unit->Neuron[RG_E].V > 0.0) {
		// 脚負荷推定(人工筋肉特性より)
		// 脚負荷 = 張力気圧特性 * 人工筋肉の伸張率
		// 	人工筋肉の伸張率 = (現在筋長 - 最小筋長) / 最大筋長	※ 最小筋長/最大筋長 = 0.66（仕様書記載の収縮率34%より）
		if(i == LF || i == RF) {  // 前脚
//			load->load_value[i] = load->load_char[i] * (Unit->Muscle[5].dL / Elbow_Extensor_Length_MAX - 0.66);
			load->load_value[i] = load->load_char[i] * (Unit->Muscle_Len[5] * 0.006667 - 0.66);
		}
		else {	// 後脚
//			load->load_value[i] = load->load_char[i] * (Unit->Muscle[5].dL / Ankle_Extensor_Length_MAX - 0.66);
			load->load_value[i] = load->load_char[i] * (Unit->Muscle_Len[5] * 0.005556 - 0.66);
		}
	}
	else if(load->flag_load[i] == 0)
		load->load_value[i] = 0.0;

	//Unit->feed_prs = load->load_value[i];
}
*/

//--------------------------------------//
//	オフセット計測(加算用)		//
//	measure_offset_sum()		//
//--------------------------------------//
void measure_offset_sum(ADType *ad)
{
	int	i;			// カウンタ
	double	ad_data[num_AD];	// ADポート出力格納先

	// 全ADポート出力値の取得
	getADall(ad_data);

	// オフセット値を加算(最終的にmeasure_offset_con()にて平均化したものをオフセット値とする)
	for(i=0; i<UNITS; i++)
		ad->POT_data_offset[i] += (float)ad_data[AD_POT_CH[i]];		// ポテンショメータ
	ad->gyro_data_offset[pitch] += (float)ad_data[AD_GYRO_CH[pitch]];	// ジャイロセンサ出力(pitch)
	ad->gyro_data_offset[roll] += (float)ad_data[AD_GYRO_CH[roll]];		// 	 　〃	     (roll)
	ad->acc_data_offset[pitch] += (float)ad_data[AD_ACC_CH[pitch]];		// 加速度センサ出力(pitch)
	ad->acc_data_offset[roll] += (float)ad_data[AD_ACC_CH[roll]];		// 	　〃	　 (roll)
}

//----------------------------------------------//
//	オフセット計測(オフセット決定用)	//
//		measure_offset_con()		//
//----------------------------------------------//
void measure_offset_con(int k, ADType *ad)
{
	int 	i;			// カウンタ
	float	acc_g[num_AXIS];	// 軸方向にかかる重力加速度[G]

	// measure_offset_sum()での総和を加算した回数kで割ることで平均化し，オフセット値として決定
	for(i=0; i<UNITS; i++) {
		ad->POT_data_offset[i] /= k;	// ポテンショメータの出力[V]
		// 足底プレートのオフセット角度を計算
		if(i == LF || i == LH) {
			ad->foot_angle_offset[i] = ad->foot_angle_last[i] = ad->POT_data_offset[i] * 330 * 0.2;
		}
		else {	// RF or RH
			ad->foot_angle_offset[i] = ad->foot_angle_last[i] = 330 - (ad->POT_data_offset[i] * 330 * 0.2);
		}
	}
	ad->gyro_data_offset[pitch] /= (float)k;	// ジャイロセンサ出力[V](pitch)
	ad->gyro_data_offset[roll] /= (float)k;		// 	   〃	　　 [V](roll)
	ad->acc_data_offset[pitch] /= (float)k;		// 加速度センサ出力[V](pitch)
	ad->acc_data_offset[roll] /= (float)k;		// 	  〃	　 [V](roll)

	// 加速度センサのオフセット出力から胴体傾きのオフセット角度を計算--------------------------------
	// 出力[V]からピッチ軸方向及びロール軸方向にかかっている重力加速度を算出(水平:0G 〜 垂直:±1G)
	acc_g[pitch] = (ad->acc_data_offset[pitch] - (2.5 * ACC_AMP)) / (ACC_SENS * ACC_AMP);
	acc_g[roll] = (ad->acc_data_offset[roll] - (2.5 * ACC_AMP)) / (ACC_SENS * ACC_AMP);

	// 胴体傾きのオフセット角度計算[rad]
	ad->tilt_angle_offset_rad[pitch] = asin(acc_g[pitch]);
	ad->tilt_angle_offset_rad[roll] = -asin(acc_g[roll]);

	// [rad] → [deg]
	ad->tilt_angle_offset_deg[pitch] = ad->tilt_angle_offset_rad[pitch] * radtodeg;
	ad->tilt_angle_offset_deg[roll] = ad->tilt_angle_offset_rad[roll] * radtodeg;
	// ----------------------------------------------------------------------------------------------
}

//--------------------------------------//
//	FlexiForce(接地判定用)		//
//		FF_get()		//
//--------------------------------------//
void all_FF_get(ADType *ad, double ad_data[])
{
	int i;

	// 電圧値取得（ 0 〜 10.24[V] ）
	for(i=0; i<UNITS; i++) {
		ad->FF_data[i] = (float)ad_data[AD_FF_CH[i]] * -1.0;	// 負の値で検出されるので，-1をかけて正の値へ

//		// 接地判定
//		if(load->FF_data[i] >= 4.0)	//(load->FF_data_offset[i] + 1.0))	//初期値+1.0以上のとき
//			load->touch[i] = 1;	// 接地
//		else
//			load->touch[i] = 0;	// 非接地
	}
}


//----------------------------------------------------------------------//
//	ポテンショメータ(3382H-1-502)から足底プレートの角度を取得	//
//				POT_get					//
//----------------------------------------------------------------------//
void all_POT_get(ADType *ad, double ad_data[], float h)
{
	int i;

	for(i=0; i<UNITS; i++) {
		// ポテンショメータの電圧[V]を取得
		ad->POT_data[i] = (float)ad_data[AD_POT_CH[i]];

		// 電圧から足底プレートの変位角度を計算(左右でポテンショメータの向きが逆)
		// 出力電圧[V] × ( ポテンショメータの最大検出角度(=330[deg]) / 最大出力電圧(=供給電圧5[V]) )
		if(i == LF || i == LH) {
			ad->foot_angle[i] = ad->POT_data[i] * 330 * 0.2;
		}
		else {	// RF or RH
			ad->foot_angle[i] = 330 - (ad->POT_data[i] * 330 * 0.2);
		}

		// オフセット角度からの相対角度を計算
		ad->foot_rel_angle[i]  = ad->foot_angle[i] - ad->foot_angle_offset[i]; // オフセット角度からの相対角度

		// 角速度の計算
		ad->foot_angle_vel[i] = ( ad->foot_angle[i] - ad->foot_angle_last[i] ) / h;

		// 一つ前の角度を更新
		ad->foot_angle_last[i] = ad->foot_angle[i];
	}
}

//----------------------//
//	接地情報取得	//
//	touch_get()	//
//----------------------//
void touch_get(ADType *ad, LOADType *load)
{
	int 	i;	// カウンタ

	// FlexiForceによる接地判定
	// LF
	if(ad->FF_data[0] >= /*ad->FF_data_offset[0] +*/ FF_touch_th)	//(load->data_FF_last[i] + 1.0))	//初期値+1.0以上のとき
		load->FF_touch[0] = 4;	// 接地
	else
		load->FF_touch[0] = 0;	// 非接地
	// LH
	if(ad->FF_data[1] >= /*ad->FF_data_offset[1] +*/ FF_touch_th)	//(load->data_FF_last[i] + 1.0))	//初期値+1.0以上のとき
		load->FF_touch[1] = 3;	// 接地
	else
		load->FF_touch[1] = 0;	// 非接地
	// RF
	if(ad->FF_data[2] >= /*ad->FF_data_offset[2] +*/ FF_touch_th)	//(load->data_FF_last[i] + 1.0))	//初期値+1.0以上のとき
		load->FF_touch[2] = 2;	// 接地
	else
		load->FF_touch[2] = 0;	// 非接地
	// RH
	if(ad->FF_data[3] >= /*ad->FF_data_offset[3] +*/ FF_touch_th)	//(load->data_FF_last[i] + 1.0))	//初期値+1.0以上のとき
		load->FF_touch[3] = 1;	// 接地
	else
		load->FF_touch[3] = 0;	// 非接地


	// ポテンショメータによる接地判定(相対角度かつ角加速度が共にほぼ0の時に非接地とする)
	// LF
	if(ad->foot_rel_angle[0] < POT_touch_th)
		load->POT_touch[0] = 0;	// 非接地
	else
		load->POT_touch[0] = 4;	// 接地
	// LH
	if(ad->foot_rel_angle[1] < POT_touch_th)
		load->POT_touch[1] = 0;	// 非接地
	else
		load->POT_touch[1] = 3;	// 接地
	// RF
	if(ad->foot_rel_angle[2] < POT_touch_th)
		load->POT_touch[2] = 0;	// 非接地
	else
		load->POT_touch[2] = 2;	// 接地
	// RH
	if(ad->foot_rel_angle[3] < POT_touch_th)
		load->POT_touch[3] = 0;	// 非接地
	else
		load->POT_touch[3] = 1;	// 接地

	// 最終的に採用する接地情報
	for(i=0; i<UNITS; i++) {
		load->touch[i] = load->FF_touch[i];
		//load->touch[i] = load->POT_touch[i];
	}
}

//----------------------------------------------------------------------//
//	ジャイロセンサ(ENC-03R)から胴体傾き角度(Pitch, Roll)を取得	//
//				gyro_get()				//
//----------------------------------------------------------------------//
void gyro_get(ADType *ad, double ad_data[], float h)
{
	// 変数宣言
	float gyro_data_deviation[num_AXIS];	// オフセット出力を基準とした出力偏差
	float gyro_angle_variation[num_AXIS];	// ひとつ前の角度からの角度変化量

	//ジャイロセンサの出力[V]取得
	ad->gyro_data[pitch] = (float)ad_data[AD_GYRO_CH[pitch]];
	ad->gyro_data[roll] = (float)ad_data[AD_GYRO_CH[roll]];

	// オフセット出力[V]を基準とした出力偏差[V]
	gyro_data_deviation[pitch] = ad->gyro_data[pitch] - ad->gyro_data_offset[pitch];
	gyro_data_deviation[roll] = ad->gyro_data[roll] - ad->gyro_data_offset[roll];

	// [V] → [mV]
	gyro_data_deviation[pitch] *= 1000.0;
	gyro_data_deviation[roll] *= 1000.0;

	// 角速度[deg/sec]の計算(= 出力偏差[mV] / 増幅後のセンサ感度[mV/deg/sec])
	ad->gyro_angle_vel[pitch] = gyro_data_deviation[pitch] / GYRO_SENS_AMP;
	ad->gyro_angle_vel[roll] = gyro_data_deviation[roll] / GYRO_SENS_AMP;

	// ひとつ前からの角度変化量[deg]の計算
	gyro_angle_variation[pitch] = ad->gyro_angle_vel[pitch] * h;
	gyro_angle_variation[roll] = ad->gyro_angle_vel[roll] * h;

	// 現在の角度更新[deg]
	ad->gyro_angle_deg[pitch] += gyro_angle_variation[pitch];	//前方正
	ad->gyro_angle_deg[roll] -= gyro_angle_variation[roll];		//方向修正,右側正

	// [deg] → [rad]
	ad->gyro_angle_rad[pitch] = ad->gyro_angle_deg[pitch] * degtorad;
	ad->gyro_angle_rad[roll] = ad->gyro_angle_deg[roll] * degtorad;

	// ローパスによる平滑化[rad]
	ad->gyro_angle_lowp_rad[pitch] = ((ad->gyro_angle_lowp_rad_last[pitch] * 399.0) + ad->gyro_angle_rad[pitch]) / 400.0;
	ad->gyro_angle_lowp_rad[roll] = ((ad->gyro_angle_lowp_rad_last[roll] * 399.0) + ad->gyro_angle_rad[roll]) / 400.0;

	// ひとつ前のローパス通した後の角度更新
	ad->gyro_angle_lowp_rad_last[pitch] = ad->gyro_angle_lowp_rad[pitch];
	ad->gyro_angle_lowp_rad_last[roll] = ad->gyro_angle_lowp_rad[roll];

	// [rad] → [deg]
	ad->gyro_angle_lowp_deg[pitch] = ad->gyro_angle_lowp_rad[pitch] * radtodeg;
	ad->gyro_angle_lowp_deg[roll] = ad->gyro_angle_lowp_rad[roll] * radtodeg;
}


//------------------------------------------------------------------------------//
//	加速度センサ(Crossbow CXL02LF3)から胴体傾き角度(Pitch, Roll)を取得	//
//				acc_get()					//
//------------------------------------------------------------------------------//
void acc_get(ADType *ad, double ad_data[], float h)
{
	// 変数宣言
	float acc[num_AXIS];		//加速度から角度への計算
	float acc_nor[num_AXIS];	//正規化

	//加速度センサの出力[V]取得
	ad->acc_data[pitch] = (float)ad_data[AD_ACC_CH[pitch]];
	ad->acc_data[roll] = (float)ad_data[AD_ACC_CH[roll]];

	// 角度変換式
	acc[pitch] = (ad->acc_data[pitch] - (2.5 * ACC_AMP)) / (2.0 * ACC_AMP);
	acc[roll] = (ad->acc_data[roll] - (2.5 * ACC_AMP)) / (2.0 * ACC_AMP);

	//正規化
	acc_nor[pitch] = acc[pitch] / 1.25;
	acc_nor[roll] = acc[roll] / 1.25;
	if(acc_nor[pitch] > 1.0){
		acc_nor[pitch] = 1.0;
	}
	if(acc_nor[pitch] < -1.0){
		acc_nor[pitch] = -1.0;
	}

	if(acc_nor[roll] > 1.0){
		acc_nor[roll] = 1.0;
	}
	if(acc_nor[roll] < -1.0){
		acc_nor[roll] = -1.0;
	}

	//角度計算
	//[rad]
	ad->acc_angle_rad[pitch] = asin(acc_nor[pitch]);
	ad->acc_angle_rad[roll] = -asin(acc_nor[roll]);
	//[deg]
	ad->acc_angle_deg[pitch] = ad->acc_angle_rad[pitch] * radtodeg;
	ad->acc_angle_deg[roll] = ad->acc_angle_rad[roll] * radtodeg;
	//inc->acc_inc_deg[pitch] = inc->acc_data[pitch];
	//inc->acc_inc_deg[roll] = inc->acc_data[roll];

	// ローパスフィルタによる平滑化[rad]
	ad->acc_angle_lowp_rad[pitch] = (ad->acc_angle_lowp_rad_last[pitch] * 199.0 + ad->acc_angle_rad[pitch]) / 200.0;
	ad->acc_angle_lowp_rad[roll] = (ad->acc_angle_lowp_rad_last[roll] * 199.0 + ad->acc_angle_rad[roll]) / 200.0;

	// ひとつ前のローパス通した後の角度更新
	ad->acc_angle_lowp_rad_last[pitch] = ad->acc_angle_lowp_rad[pitch];	//更新
	ad->acc_angle_lowp_rad_last[roll] = ad->acc_angle_lowp_rad[roll];

	// [rad] → [deg]
	ad->acc_angle_lowp_deg[pitch] = ad->acc_angle_lowp_rad[pitch] * radtodeg;
	ad->acc_angle_lowp_deg[roll] = ad->acc_angle_lowp_rad[roll] * radtodeg;
}



//------------------------------//
//	胴体傾き角度の決定	//
//	    tilt_con()		//
//------------------------------//
void tilt_con(ADType *ad)
{
	// 変数宣言
	float drift[num_AXIS];	//ドリフト値

	// ドリフト値を計算
	drift[pitch] = ad->gyro_angle_lowp_rad[pitch] - ad->acc_angle_lowp_rad[pitch];
	drift[roll] = ad->gyro_angle_lowp_rad[roll] - ad->acc_angle_lowp_rad[roll];

	// 胴体傾斜角度の計算[rad]
	ad->tilt_angle_rad[pitch] = ad->gyro_angle_rad[pitch] - drift[pitch];
	ad->tilt_angle_rad[roll] = ad->gyro_angle_rad[roll] - drift[roll];

	// [rad] → [deg]
	ad->tilt_angle_deg[pitch] = ad->tilt_angle_rad[pitch] * radtodeg;
	ad->tilt_angle_deg[roll] = ad->tilt_angle_rad[roll] * radtodeg;
}


//----------------------//
//	空気圧取得	//
//	air_press_get()	//
//----------------------//
void air_press_get(ADType *ad, double ad_data[])
{

	// 気圧センサ値[V]を取得
	ad->air_press_data = (float)(ad_data[AD_AIR_PRESS_CH]);

	// 気圧[MPa]へ変換
	// 変換式は空気圧センサ(PSE530-M5-L)の正圧用アナログ出力の特性グラフより導出
	ad->atmosphere = 0.25 * ad->air_press_data - 0.25;
}
