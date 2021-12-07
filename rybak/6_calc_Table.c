// -------------------------------------------- //
//	ルックアップテーブル関連関数群		//
//	calc_Table.c				//
//	(calculation_Table)			//
// -------------------------------------------- //

#include "8_define.h"
#include "9_user_param.h"


// 高速計算用テーブル宣言
float Muscle6_F_len_table[Muscle_len_table_num][Muscle_len_table_num];
float Muscle6_H_len_table[Muscle_len_table_num][Muscle_len_table_num];
float mK_table[V_num];
float mNaP_table[V_num];
float hInfNaP_table[V_num];
//float tauhNaP_RG_table[V_num];
float tauhNaP_RG_inv_table[V_num];
//float tauhNaP_table[V_num];
float tauhNaP_inv_table[V_num];
float fV_Mn_table[V_num];
float fV_table[V_num];


void Muscle_len_table_init()
{
	int i, j;
	float bx1, by1, bx2, by2, dist_x, dist_y;
	float shoulder_rad, elbow_rad, knee_rad, ankle_rad;

	// 前脚 ------------------------------------------------------------------------------------------------------------------
	// 単関節筋
/*	for (i=0; i<Muscle_len_table_num; i++) {
		scapula_rad = ( Scapula_MIN + i * (Scapula_MAX - Scapula_MIN) / (Muscle_len_table_num - 1) ) * degtorad;
		shoulder_rad = ( Shoulder_MIN + i * (Shoulder_MAX - Shoulder_MIN) / (Muscle_len_table_num - 1) ) * degtorad;
		elbow_rad = ( Elbow_MIN + i * (Elbow_MAX - Elbow_MIN) / (Muscle_len_table_num - 1) ) * degtorad;

		// 1番筋長さ（肩甲骨屈筋）
		bx1 = cos(-scapula_rad) * (Link1_F_Length + Muscle1_F_Connect_bx) - sin(-scapula_rad) * Muscle1_F_Connect_by;
		by1 = sin(-scapula_rad) * (Link1_F_Length + Muscle1_F_Connect_bx) + cos(-scapula_rad) * Muscle1_F_Connect_by;
		Muscle1_F_len_table[i] = sqrt(pow2(bx1 - Muscle1_F_Connect_ax) + pow2(by1 - Muscle1_F_Connect_ay));
		// 2番筋長さ（肩甲骨伸筋）
		bx1 = cos(-scapula_rad) * (Link1_F_Length + Muscle2_F_Connect_bx) - sin(-scapula_rad) * Muscle2_F_Connect_by;
		by1 = sin(-scapula_rad) * (Link1_F_Length + Muscle2_F_Connect_bx) + cos(-scapula_rad) * Muscle2_F_Connect_by;
		Muscle2_F_len_table[i] = sqrt(pow2(bx1 - Muscle2_F_Connect_ax) + pow2(by1 - Muscle2_F_Connect_ay));
		// 4番筋長さ（肩伸筋）
		bx1 = cos(-(PI-shoulder_rad)) * Muscle4_F_Connect_bx - sin(-(PI-shoulder_rad)) * Muscle4_F_Connect_by + Link1_F_Length;
		by1 = sin(-(PI-shoulder_rad)) * Muscle4_F_Connect_bx + cos(-(PI-shoulder_rad)) * Muscle4_F_Connect_by;
		Muscle4_F_len_table[i] = sqrt(pow2(bx1 - Muscle4_F_Connect_ax) + pow2(by1 - Muscle4_F_Connect_ay));
		// 5番筋長さ（肘屈筋）
		bx1 = cos((PI-elbow_rad)) * Muscle5_F_Connect_bx - sin((PI-elbow_rad)) * Muscle5_F_Connect_by + Link2_F_Length;
		by1 = sin((PI-elbow_rad)) * Muscle5_F_Connect_bx + cos((PI-elbow_rad)) * Muscle5_F_Connect_by;
		Muscle5_F_len_table[i] = sqrt(pow2(bx1 - Muscle5_F_Connect_ax) + pow2(by1 - Muscle5_F_Connect_ay));	
	}
*/
	// 二関節筋
/*	for (i=0; i<Muscle_len_table_num; i++) {
		scapula_rad = ( Scapula_MIN + i * (Scapula_MAX - Scapula_MIN) / (Muscle_len_table_num - 1) ) * degtorad;
		for(j=0; j<Muscle_len_table_num; j++) {
			shoulder_rad = ( Shoulder_MIN + j * (Shoulder_MAX - Shoulder_MIN) / (Muscle_len_table_num - 1) ) * degtorad;

			// 3番筋長さ（肩屈筋）
			bx1 = cos(-(PI-shoulder_rad)) * Muscle3_F_Connect_bx - sin(-(PI-shoulder_rad)) * Muscle3_F_Connect_by + Link1_F_Length;
			by1 = sin(-(PI-shoulder_rad)) * Muscle3_F_Connect_bx + cos(-(PI-shoulder_rad)) * Muscle3_F_Connect_by;
			bx2 = cos(-scapula_rad) * bx1 - sin(-scapula_rad) * by1;
			by2 = sin(-scapula_rad) * bx1 + cos(-scapula_rad) * by1;
			Muscle3_F_len_table[i][j] = sqrt(pow2(bx2 - Muscle3_F_Connect_ax) + pow2(by2 - Muscle3_F_Connect_ay));
		}
	}
*/
	for (i=0; i<Muscle_len_table_num; i++) {
		shoulder_rad = ( ANGLE_SHOULDER_MIN + i * (ANGLE_SHOULDER_MAX - ANGLE_SHOULDER_MIN) / (Muscle_len_table_num - 1) ) * degtorad;
		for(j=0; j<Muscle_len_table_num; j++) {
			elbow_rad = ( ANGLE_ELBOW_MIN + j * (ANGLE_ELBOW_MAX - ANGLE_ELBOW_MIN) / (Muscle_len_table_num - 1) ) * degtorad;

			// 6番筋長さ（肘伸筋）
			bx1 = cos((PI-elbow_rad)) * Muscle6_F_Connect_bx - sin((PI-elbow_rad)) * Muscle6_F_Connect_by + Link2_F_Length;
			by1 = sin((PI-elbow_rad)) * Muscle6_F_Connect_bx + cos((PI-elbow_rad)) * Muscle6_F_Connect_by;
			bx2 = cos(-(PI-shoulder_rad)) * bx1 - sin(-(PI-shoulder_rad)) * by1;
			by2 = sin(-(PI-shoulder_rad)) * bx1 + cos(-(PI-shoulder_rad)) * by1;
			dist_x = bx2 - Muscle6_F_Connect_ax;
			dist_y = by2 - Muscle6_F_Connect_ay;
			Muscle6_F_len_table[i][j] = sqrt(pow2(dist_x) + pow2(dist_y));
		}
	}
	// -----------------------------------------------------------------------------------------------------------------------

	// 後脚 ------------------------------------------------------------------------------------------------------------------
	// 単関節筋
/*	for (i=0; i<Muscle_len_table_num; i++) {
		hip_rad = ( Hip_MIN + i * (Hip_MAX - Hip_MIN) / (Muscle_len_table_num - 1) ) * degtorad;
		knee_rad = ( Knee_MIN + i * (Knee_MAX - Knee_MIN) / (Muscle_len_table_num - 1) ) * degtorad;
		ankle_rad = ( Ankle_MIN + i * (Ankle_MAX - Ankle_MIN) / (Muscle_len_table_num - 1) ) * degtorad;

		// 1番筋長さ（腰屈筋）
		bx1 = cos(-hip_rad) * (Link1_H_Length + Muscle1_H_Connect_bx) - sin(-hip_rad) * Muscle1_H_Connect_by;
		by1 = sin(-hip_rad) * (Link1_H_Length + Muscle1_H_Connect_bx) + cos(-hip_rad) * Muscle1_H_Connect_by;
		Muscle1_H_len_table[i] = sqrt(pow2(bx1 - Muscle1_H_Connect_ax) + pow2(by1 - Muscle1_H_Connect_ay));
		// 2番筋長さ（腰伸筋）
		bx1 = cos(-hip_rad) * (Link1_H_Length + Muscle2_H_Connect_bx) - sin(-hip_rad) * Muscle2_H_Connect_by;
		by1 = sin(-hip_rad) * (Link1_H_Length + Muscle2_H_Connect_bx) + cos(-hip_rad) * Muscle2_H_Connect_by;
		Muscle2_H_len_table[i] = sqrt(pow2(bx1 - Muscle2_H_Connect_ax) + pow2(by1 - Muscle2_H_Connect_ay));
		// 4番筋長さ（膝伸筋）
		bx1 = cos(-(PI-knee_rad)) * Muscle4_H_Connect_bx - sin(-(PI-knee_rad)) * Muscle4_H_Connect_by + Link1_H_Length;
		by1 = sin(-(PI-knee_rad)) * Muscle4_H_Connect_bx + cos(-(PI-knee_rad)) * Muscle4_H_Connect_by;
		Muscle4_H_len_table[i] = sqrt(pow2(bx1 - Muscle4_H_Connect_ax) + pow2(by1 - Muscle4_H_Connect_ay));
		// 5番筋長さ（足首屈筋）
		bx1 = cos((PI-ankle_rad)) * Muscle5_H_Connect_bx - sin((PI-ankle_rad)) * Muscle5_H_Connect_by + Link2_H_Length;
		by1 = sin((PI-ankle_rad)) * Muscle5_H_Connect_bx + cos((PI-ankle_rad)) * Muscle5_H_Connect_by;
		Muscle5_H_len_table[i] = sqrt(pow2(bx1 - Muscle5_H_Connect_ax) + pow2(by1 - Muscle5_H_Connect_ay));	
	}
*/
	// 二関節筋
/*	for (i=0; i<Muscle_len_table_num; i++) {
		hip_rad = ( Hip_MIN + i * (Hip_MAX - Hip_MIN) / (Muscle_len_table_num - 1) ) * degtorad;
		for(j=0; j<Muscle_len_table_num; j++) {
			knee_rad = ( Knee_MIN + j * (Knee_MAX - Knee_MIN) / (Muscle_len_table_num - 1) ) * degtorad;

			// 3番筋長さ（膝屈筋）
			bx1 = cos(-(PI-knee_rad)) * Muscle3_H_Connect_bx - sin(-(PI-knee_rad)) * Muscle3_H_Connect_by + Link1_H_Length;
			by1 = sin(-(PI-knee_rad)) * Muscle3_H_Connect_bx + cos(-(PI-knee_rad)) * Muscle3_H_Connect_by;
			bx2 = cos(-hip_rad) * bx1 - sin(-hip_rad) * by1;
			by2 = sin(-hip_rad) * bx1 + cos(-hip_rad) * by1;
			Muscle3_H_len_table[i][j] = sqrt(pow2(bx2 - Muscle3_H_Connect_ax) + pow2(by2 - Muscle3_H_Connect_ay));
		}
	}
*/
	for (i=0; i<Muscle_len_table_num; i++) {
		knee_rad = ( ANGLE_KNEE_MIN + i * (ANGLE_KNEE_MAX - ANGLE_KNEE_MIN) / (Muscle_len_table_num - 1) ) * degtorad;
		for(j=0; j<Muscle_len_table_num; j++) {
			ankle_rad = ( ANGLE_ANKLE_MIN + j * (ANGLE_ANKLE_MAX - ANGLE_ANKLE_MIN) / (Muscle_len_table_num - 1) ) * degtorad;

			// 6番筋長さ（足首伸筋）
			bx1 = cos((PI-ankle_rad)) * Muscle6_H_Connect_bx - sin((PI-ankle_rad)) * Muscle6_H_Connect_by + Link2_H_Length;
			by1 = sin((PI-ankle_rad)) * Muscle6_H_Connect_bx + cos((PI-ankle_rad)) * Muscle6_H_Connect_by;
			bx2 = cos(-(PI-knee_rad)) * bx1 - sin(-(PI-knee_rad)) * by1;
			by2 = sin(-(PI-knee_rad)) * bx1 + cos(-(PI-knee_rad)) * by1;
			dist_x = bx2 - Muscle6_H_Connect_ax;
			dist_y = by2 - Muscle6_H_Connect_ay;
			Muscle6_H_len_table[i][j] = sqrt(pow2(dist_x) + pow2(dist_y));
		}
	}
	// -----------------------------------------------------------------------------------------------------------------------
}

// 筋長計算の高速化のためテーブル参照を利用
// 前脚の6番筋の筋長計算
float Calc_Muscle6_F_len(float angle_shoulder, float angle_elbow)
{
	int i, j;

	if (angle_shoulder < ANGLE_SHOULDER_MIN) angle_shoulder = ANGLE_SHOULDER_MIN;
	if (angle_shoulder > ANGLE_SHOULDER_MAX) angle_shoulder = ANGLE_SHOULDER_MAX;
	if (angle_elbow < ANGLE_ELBOW_MIN) angle_elbow = ANGLE_ELBOW_MIN;
	if (angle_elbow > ANGLE_ELBOW_MAX) angle_elbow = ANGLE_ELBOW_MAX;
//	i = (int)( (angle_shoulder - ANGLE_SHOULDER_MIN)/(ANGLE_SHOULDER_MAX - ANGLE_SHOULDER_MIN) * (Muscle_len_table_num - 1) );
	i = (int)( (angle_shoulder - 80.0) * 2.5);	// ANGLE_SHOULDER_MIN = 80.0, ANGLE_SHOULDER_MAX = 120.0, Muscle_len_table_num = 101の時
//	j = (int)( (angle_elbow - ANGLE_ELBOW_MIN)/(ANGLE_ELBOW_MAX - ANGLE_ELBOW_MIN) * (Muscle_len_table_num - 1) );
	j = (int)( (angle_elbow - 65.0) * 1.666667);	// ANGLE_ELBOW_MIN = 65.0, ANGLE_ELBOW_MAX = 125.0, Muscle_len_table_num = 101の時

	return Muscle6_F_len_table[i][j];
}

// 後脚の6番筋の筋長計算
float Calc_Muscle6_H_len(float angle_knee, float angle_ankle)
{
	int i, j;

	if (angle_knee < ANGLE_KNEE_MIN) angle_knee = ANGLE_KNEE_MIN;
	if (angle_knee > ANGLE_KNEE_MAX) angle_knee = ANGLE_KNEE_MAX;
	if (angle_ankle < ANGLE_ANKLE_MIN) angle_ankle = ANGLE_ANKLE_MIN;
	if (angle_ankle > ANGLE_ANKLE_MAX) angle_ankle = ANGLE_ANKLE_MAX;
//	i = (int)( (angle_knee - ANGLE_KNEE_MIN)/(ANGLE_KNEE_MAX - ANGLE_KNEE_MIN) * (Muscle_len_table_num - 1) );
	i = (int)( (angle_knee - 75.0) * 2.222222 + 0.00001);	// ANGLE_KNEE_MIN = 75.0, ANGLE_KNEE_MAX = 120.0, Muscle_len_table_num = 101の時
//	j = (int)( (angle_ankle - ANGLE_ANKLE_MIN)/(ANGLE_ANKLE_MAX - ANGLE_ANKLE_MIN) * (Muscle_len_table_num - 1) );
	j = (int)( (angle_ankle - 40.0) * 1.25);	// ANGLE_ANKLE_MIN = 40.0, ANGLE_ANKLE_MAX = 120.0, Muscle_len_table_num = 101の時

	return Muscle6_H_len_table[i][j];
}


// ニューロン内パラメータ計算の高速化のためテーブル参照を利用
// テーブル作成
void Aux_table_init()
{
	int i;
	float V;

	for (i=0; i<V_num; i++) {
//		V = V_MIN + i * (V_MAX - V_MIN) / (V_num - 1);
		V = -80.0 + i * 0.11;	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001の時

		mK_table[i] = 1.0 / (1.0 + exp( -(V + 44.5) * 0.2) );
		mNaP_table[i] = 1.0 / (1.0 + exp( -(V + 47.1) / 3.1) );
		hInfNaP_table[i] = 1.0 / (1.0 + exp( (V + 51.0) * 0.25) );

//		tauhNaP_RG_table[i] = 300.0e-3 / cosh( (V + 51.0) * 0.125 );
//		tauhNaP_table[i] = 600.0 / cosh( (V+51.0) * 0.125 );
		tauhNaP_RG_inv_table[i] = cosh( (V + 51.0) * 0.125 ) / 300.0e-3;
		tauhNaP_inv_table[i] = cosh( (V + 51.0) * 0.125 ) / 600.0;

	}
}


float Calc_mK(int i)
{
	return mK_table[i];
}

float Calc_mNaP(int i)
{
	return mNaP_table[i];
}

float Calc_hInfNaP(int i)
{
	return hInfNaP_table[i];
}

/*
float Calc_tauhNaP_RG(int i)
{
	return tauhNaP_RG_table[i];
}
*/

// tauhNaP_RGの逆数
float Calc_tauhNaP_RG_inv(int i)
{
	return tauhNaP_RG_inv_table[i];
}
/*
float Calc_tauhNaP(int i)//float y)
{
	return tauhNaP_table[i];
}
*/
// tauhNaPの逆数
float Calc_tauhNaP_inv(int i)
{
	return tauhNaP_inv_table[i];
}

void fV_table_init()
{
	int i;
	float V;

	for (i=0; i<V_num; i++) {
//		V = V_MIN + i * (V_MAX - V_MIN) / (V_num - 1);
		V = -80.0 + i * 0.11;	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001の時
		if (V >= -50.0) {//Vth) {
			fV_Mn_table[i] = 1.0 / ( 1.0 + exp( -(V + 40.0) * 0.5 ) );
			fV_table[i] = 1.0 / ( 1.0 + exp( -(V + 30.0) * 0.125 ) );
		}
		else {
			fV_Mn_table[i] = 0.0;
			fV_table[i] = 0.0;
		}
	}
}


float Calc_fV_Mn(int i)
{
	return fV_Mn_table[i];
}

float Calc_fV(int i)
{
	return fV_table[i];
}
