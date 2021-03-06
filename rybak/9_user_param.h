/*変更する主なパラメータをここにまとめる
　　注意：ここで値を設定していても本文の中で書き換えている可能性があるためよく確認すること．*/

#ifndef __USER_PARAM_HEADER_INCLUDED__
#define __USER_PARAM_HEADER_INCLUDED__

// 各種実行期間 --------------------------------------------------------------------------
//#define Time_Loop	(35.0)	// メインループ時間[s]（歩行終了後も少しだけ計算を行う）
#define Time_Locomotion	(30.0)	// 歩行時間[s]
#define Time_Reset 	(5.0/*5.0*/)	// 全筋肉弛緩時間[s]
#define Time_Flexion	(5.0)	// 全関節屈曲時間[s]
#define Time_Extension	(5.0)	// 全関節伸展時間[s]
#define Time_Offset	(1.0)	// オフセット計測時間[s]
#define TIME_STEP 	(1000)	// サンプリングタイム（1000[μsec] → 1[msec]）

// 各種経過時間設定 ----------------------------------------------------------------------
// CPG波形を安定させるため，歩行準備（弛緩→屈曲→弛緩）中もCPG計算は実行しておく
#define Time_Step1	(Time_Offset)			// 歩行準備（オフセット計測）
#define Time_Step2	(Time_Step1 + Time_Reset)	// 歩行準備（一度目の全筋肉弛緩）
#define Time_Step3	(Time_Step2 + Time_Flexion)	// 歩行準備（全関節屈曲&エンコーダリセット）
#define Time_Step4	(Time_Step3 + Time_Reset)	// 歩行準備（二度目の全筋肉弛緩）
#define Time_Step5	(Time_Step4 + Time_Locomotion)	// 歩行（Time_Step4:歩行開始時刻に相当）
#define Time_Step6	(Time_Step5 + Time_Reset)	// 歩行後の全筋肉弛緩

// パラメータ変更のタイミング設定（Time_State1 < Time_State となるように）
#define Time_State1	(Time_Step4 + 10.0)	// パラメータ変更開始時刻
#define Time_State2	(Time_Step4 + 20.0)	// パラメータ変更終了時刻

// 脚間結合荷重 --------------------------------------------------------------------------
#define U2U_FH	(0.04/*/0.038*/)	// 抑制性前後間
#define U2U_LR	(0.04/*/0.032*/)	// 抑制性左右間

#define U1U_FH	(0.05/*/0.038*/)	// 興奮性前後間
#define U1U_LR	(0.05/*/0.032*/)	// 興奮性左右間

// 上位入力設定 --------------------------------------------------------------------------
#define Sup_Driven1	(1.56)//元は2.00 (1.70)//*/(1.96)	// 上位入力d[0]（パラメータ変更前）2019/01/15 フィードバック無し:
#define Sup_Driven2	(1.56)//元は2.00 (2.68)//*/(1.96)	// 上位入力d[0]（パラメータ変更後）		下限1.56程度・・・周期0.54
#define Ext_Driven1	(0.0)	// 外部入力d[1]（パラメータ変更前）
#define Ext_Driven2	(0.0)	// 外部入力d[1]（パラメータ変更後）

// 接地判定に用いるFF電圧値の閾値
#define FF_touch_th	(3.0)

// 接地判定に用いる足底プレート相対角度の閾値
#define POT_touch_th	(5.0)

// 筋張力特性設定（筋長の変位割合に対する張力） [N]→[kN]に変換した ------------------------------------------
/***パラメータ目安****************************************
        (1.28)  0.3[MPaG]
        (1.56)  0.35[MPaG]
        (1.85)  0.4[MPaG]
        (2.17)  0.45[MPaG]
        (2.49)  0.5[MPaG]
        (2.66)  0.55[MPaG]
        (2.84)  0.6[MPaG]
********************************************************/
//前脚
#define Load_char1_F	(2.49) //(2.49) 		//0.53MPaG	//0.3MPaG:1.28, 0.4MPaG:1.85
#define Load_char2_F	(2.49) //(2.84)		//0.65MPaG	//0.5MPaG:2.49, 0.6MPaG:2.84
//後脚
#define Load_char1_H	(Load_char1_F) //(2.49)		//0.53MPaG	//0.3MPaG:1.28, 0.4MPaG:1.85
#define Load_char2_H	(Load_char2_F) //(2.84)		//0.65MPaG	//0.5MPaG:2.49, 0.6MPaG:2.84//#define Load_char2	(2.8402)	// パラメータ変更後(0.6[MPaG])

// 脚負荷ゲイン（前脚と後脚で脚負荷の大きさが異なるので要調整） --------------------------
#define load_gain_H	(20.0)	//(20.0)	// 後脚
#define load_gain_F	(load_gain_H) 	// 前脚

#define sigmoid_alpha	(3.00) //0.75

// Mnの振幅による電磁弁制御 閾値 --------------------------
// 電磁弁を駆動させるMn発火率の閾値
#define Mn_fVth		(0.03)
//電磁弁解放段階の閾値
#define Mn_phase0 (0.03)    //[0 0 0]と[1 0 0]の境界
#define Mn_phase1 (0.07)    //[1 0 0]と[1 1 0]の境界
#define Mn_phase2 (0.17)    //[1 1 0]と[1 1 1]の境界

// 計算高速化のために使用するパラメータ
#define	Muscle_len_table_num	(101)	// 筋長計算テーブルの分割数
#define V_num	(1001)	// ニューロン内パラメータ計算テーブルの分割数
#define V_MAX	(30.0)	// ニューロン膜電位の最大値
#define V_MIN	(-80.0)	// ニューロン膜電位の最小値


// ↓↓↓ ハードに変更があった場合 ↓↓↓
// 各リンク長[mm] ------------------------------------------------------------------------
#define Link1_F_Length	(64.0)	// 前脚リンク1
#define Link2_F_Length	(130.0)	// 前脚リンク2
#define Link3_F_Length	(161.0)	// 前脚リンク3

#define Link1_H_Length	(150.0)	// 後脚リンク1
#define Link2_H_Length	(150.0)	// 後脚リンク2
#define Link3_H_Length	(91.5)	// 後脚リンク3

// 筋肉の取り付け位置（関節軸との距離[mm]） ----------------------------------------------
// 前脚1番筋
#define Muscle1_F_Connect_ax	(20.0)
#define Muscle1_F_Connect_ay	(95.0)
#define Muscle1_F_Connect_bx	(-20.0)
#define Muscle1_F_Connect_by	(10.0)
// 前脚2番筋
#define Muscle2_F_Connect_ax	(-130.0)
#define Muscle2_F_Connect_ay	(55.0)
#define Muscle2_F_Connect_bx	(-30.0)
#define Muscle2_F_Connect_by	(-10.0)
// 前脚3番筋
#define Muscle3_F_Connect_ax	(-45.0)
#define Muscle3_F_Connect_ay	(60.0)
#define Muscle3_F_Connect_bx	(18.0)
#define Muscle3_F_Connect_by	(0.0)
// 前脚4番筋
#define Muscle4_F_Connect_ax	(-60.0)
#define Muscle4_F_Connect_ay	(20.0)
#define Muscle4_F_Connect_bx	(-30.0)
#define Muscle4_F_Connect_by	(10.0)
// 前脚5番筋
#define Muscle5_F_Connect_ax	(110.0)
#define Muscle5_F_Connect_ay	(3.0)
#define Muscle5_F_Connect_bx	(130.0)
#define Muscle5_F_Connect_by	(10.0)
// 前脚6番筋
#define Muscle6_F_Connect_ax	(-15.0)
#define Muscle6_F_Connect_ay	(-8.0)
#define Muscle6_F_Connect_bx	(-20.0)
#define Muscle6_F_Connect_by	(-11.0)
// 後脚1番筋
#define Muscle1_H_Connect_ax	(20.0)
#define Muscle1_H_Connect_ay	(0.0)
#define Muscle1_H_Connect_bx	(-17.0)
#define Muscle1_H_Connect_by	(9.5)
// 後脚2番筋
#define Muscle2_H_Connect_ax	(-50.0)
#define Muscle2_H_Connect_ay	(4.0)
#define Muscle2_H_Connect_bx	(-27.0)
#define Muscle2_H_Connect_by	(-10.0)
// 後脚3番筋
#define Muscle3_H_Connect_ax	(-10.0)
#define Muscle3_H_Connect_ay	(10.0)
#define Muscle3_H_Connect_bx	(21.0)
#define Muscle3_H_Connect_by	(0.0)
// 後脚4番筋
#define Muscle4_H_Connect_ax	(-15.0)
#define Muscle4_H_Connect_ay	(12.0)
#define Muscle4_H_Connect_bx	(-27.0)
#define Muscle4_H_Connect_by	(0.0)
// 後脚5番筋
#define Muscle5_H_Connect_ax	(21.0)
#define Muscle5_H_Connect_ay	(0.0)
#define Muscle5_H_Connect_bx	(20.0)
#define Muscle5_H_Connect_by	(2.0)
// 後脚6番筋
#define Muscle6_H_Connect_ax	(-13.0)
#define Muscle6_H_Connect_ay	(-5.0)
#define Muscle6_H_Connect_bx	(-27.0)
#define Muscle6_H_Connect_by	(-10.0)


// 第3関節伸筋の最大筋長 -----------------------------------------------------------------
#define Elbow_Extensor_Length_MAX	(150)	// 肘伸筋
#define Ankle_Extensor_Length_MAX	(180)	// 足首伸筋

//各関節の最小角度（最大屈曲時）[deg] ----------------------------------------------------
// 前脚
#define	ANGLE_SCAPULA_MIN	(40.0)	// 肩甲骨
#define	ANGLE_SHOULDER_MIN	(80.0)	// 肩
#define	ANGLE_ELBOW_MIN		(65.0)	// 肘
// 後脚
#define	ANGLE_HIP_MIN		(40.0)	// 腰
#define	ANGLE_KNEE_MIN		(75.0)	// 膝
#define	ANGLE_ANKLE_MIN		(40.0)	// 足首

//各関節の最大角度（最大伸展時）[deg] ----------------------------------------------------
// 前脚
#define ANGLE_SCAPULA_MAX	(85.0)	// 肩甲骨
#define	ANGLE_SHOULDER_MAX	(120.0)	// 肩
#define	ANGLE_ELBOW_MAX		(125.0)	// 肘
// 後脚
#define	ANGLE_HIP_MAX		(105.0)	// 腰
#define	ANGLE_KNEE_MAX		(120.0)	// 膝
#define	ANGLE_ANKLE_MAX		(120.0)	// 足首



#endif
