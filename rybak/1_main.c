// ---------------------------------------------------- //
//	プログラム"Rybak_robot"のメインプログラム	//
//			main.c				//
// ---------------------------------------------------- //

#include "8_define.h"
#include "9_user_param.h"

int main(void)
{
	//------変数宣言と初期化------//
	int		DIO_MODE;				// DIOモード設定用
	double		AD_data[num_AD];			// ADポート出力の格納先(HRPのAPI仕様に基づきdouble型)
	int		i;					// 汎用カウンタ
	int		k_offset = 0;				// オフセット計測用カウンタ
	int		k_sam = 0;				// サンプルデータ用カウンタ
	int		k_func = 0;				// 関数実行回数用カウンタ
	float		d[2] = {Sup_Driven1, Ext_Driven1};	// 上位入力
	float		t = 0.0;				// 歩行開始後の経過時間[sec]
	float		t_last = 0.0;				// メインループの最終実行時刻[sec]
	float		t_last_1 = 0.0;				// 分岐1の最終実行時刻[sec]
    float		t_last_2 = 0.0;				// 分岐2の最終実行時刻[sec]
	float 		ts = 0.0;				// 歩行開始時刻
	float 		tn = 0.0;				// 現在時刻
	float		t1;					// 関数処理直前の時刻 (関数処理時間計測用)
	float		t2;					// 関数処理直後の時刻	      〃
	float		t_proc;					// 関数の処理時間[s]
	float		t_proc_sum = 0.0;			// 関数の合計処理時間[s]
	float		t_proc_max = 0.0;			// 関数の最大処理時間[s]
	double		tmu_frq = tsc_clock();			// タイマユニットの周波数
	int		flag_step = 0;				// ループ内のステップ切り替えフラグ

	// 各種構造体宣言
	RybakUnit	unit[UNITS];
	ENCType		enc;
	ADType		ad;
	SAMType		sam;
	LOADType	load;

	// ユニット初期化
	InitUnit(unit);

	//------メッセージ表示------//
	printf("\t \t Program Start!\n");	// プログラム開始メッセージ表示

	// 計算高速化のためのテーブル作成
	printf("\t \t Make Look-Up Tables\n");
	Muscle_len_table_init();
	Aux_table_init();
	fV_table_init();

	//------開始処理------//
	openHRP3sh4();	// HRPデバイス使用開始
	tsc_open();	// タイマユニットカウンタをユーザへ

	//------リアルタイム制御開始------//
	// ART_PRIO_MAX:最優先, ART_TASK_PERIODIC:周期実行, TIME_STEP:サンプリングタイム
	if(art_enter(ART_PRIO_MAX, ART_TASK_PERIODIC, TIME_STEP) != 0){
		perror("art_enter");
		return -1;
	}

	//------A/D変換開始------//
	startAD();

	//------初期化------//
	// PWMの初期化
	for (i=0; i<num_PWM; i++) {
		setPWMmode(i, ON);	// PWMを全てON
		setPWMduty(i, 0.0);
	}
	// DIOの初期化
	DIO_MODE = 0xffffffff;
	setDIOmodeAll32(DIO_MODE);	// DIOモードの設定
	for (i=0; i<num_DIO; i++)
		setDIO(i, OFF);		// DIOを全てOFF

	// 比例弁作動（上手く通信できない時があるので，複数回実行しておく）
	udp_transmission(t);
	udp_transmission(t);
	udp_transmission(t);

	ts = tn = (float)rdtsc(); // ループ開始時刻設定

	// メインループ ////////////////////////////////////////////////////////////////////////////////////////////////
	while (1) {
		t1 = (float)rdtsc();				// 関数処理直前の時刻

		//----周期実行基準時刻の取得----//
		if(art_wait() != 0)
			perror("art_wait");

		// 時刻更新
		tn = (float)rdtsc();	// 現在時刻取得
		t = (float)((tn-ts)/tmu_frq/1.0e+6) + 2.0e-3;	// 経過時間の更新


		// ステップ用フラグの更新＆ステップ移行メッセージの表示 --------------- //
		// ステップ1（オフセット計測期間）へ移行
		if (flag_step == 0) {
			flag_step++;
			printf("t = %f\n", t);
			printf("\t \t Measuring Offsets...\n\n");
		}
		// オフセットを決定しつつステップ2（全筋肉弛緩）へ移行
		if(flag_step == 1 && t > Time_Step1) {
			measure_offset_con(k_offset, &ad);	// オフセット計測(オフセット決定用)
			flag_step++;
			printf("t = %f\n", t);
			printf("\t \t Resetting Muscles...\n\n");
		}
		// エンコーダ初期化のためにステップ3（全関節屈曲）へ移行
		if (flag_step == 2 && t > Time_Step2) {
			flag_step++;
			printf("t = %f\n", t);
			printf("\t \t Setting Encoder...\n\n");
		}
		// エンコーダ初期化を行いつつステップ4（全筋肉弛緩）へ移行
		if (flag_step == 3 && t > Time_Step3) {
			for(i=0; i<num_ENC; i++)
				setEnc(i,0);	// エンコーダ初期化
			flag_step++;
			printf("t = %f\n", t);
			printf("\t \t Resetting Muscles...\n\n");
		}
		// ステップ5（歩行運動）へ移行
		if (flag_step == 4 && t > Time_Step4) {
			flag_step++;
			printf("t = %f\n", t);
			printf("\t \t Locomotion Start!\n\n");
			t_last_1 = t;
            t_last_2 = t;
		}
		// 歩行運動を終了し，ステップ6（全筋肉弛緩）へ移行
		if (flag_step == 5 && t > Time_Step5) {
			udp_transmission(t);	// 比例弁停止（念のため複数回実行）
			udp_transmission(t);
			udp_transmission(t);
			printf("t = %f\n", t);
			printf("\t \t Locomotion End.\n\n");
			flag_step++;
			printf("\t \t Resetting Muscles...\n\n");
		}
		// メインループから抜け出す
		if (flag_step == 6 && t > Time_Step6)
			break;
		// -------------------------------------------------------------------- //


		// CPG内の各ニューロン状態を更新（ステップに関わらず毎ループ実行）
		//↓の2次か4次どちらか一方はコメントアウトすること
		CalcUnit_RK2(unit, d, t-t_last, &enc, &load);	// 2次ルンゲクッタ
//		CalcUnit_RK4(unit, d, t-t_last, &enc, &load);	// 4次ルンゲクッタ
		CalcCycle(unit, t);	// CPG周期の計算


		// ステップ毎にループ ------------------------------------------------------------------------
		switch (flag_step) {
			// ------ ステップ1（オフセット計測期間） ------ //
			case 1:
				measure_offset_sum(&ad);	// オフセット計測(加算用)
				k_offset++;				// 加算回数更新
				break;
			// ------ ステップ2（全筋肉弛緩1） ------ //
			case 2:
				muscle_reset();		// 全筋肉弛緩
				break;
			// ------ ステップ3（全関節屈曲） ------ //
			case 3:
				muscle_flexion();	// 全関節屈曲
				break;
			// ------ ステップ4（全筋肉弛緩2） ------ //
			case 4:
				muscle_reset();		// 全筋肉弛緩
				break;
			// ------ ステップ5（歩行運動） ------ //
			case 5:
				// 筋肉制御
				//control_Muscles(unit);
				// ループのタイミングで実行内容を分岐---------------------------------- //
				//	→ 計算周期を2[ms]に収めるため同時には実行しない
				// 分岐1：　10[ms]毎に実行
				if(1.0e-2 < t - t_last_1){
//					// サンプルデータの保存
					data_get(k_sam, t, &sam, unit, &enc, &ad, &load); // データの格納
					k_sam++;	// サンプルデータの要素数更新
					// 時間経過によるパラメータ設定
					param(d, &load, t);
					// 時間経過による比例弁のduty比設定
					udp_transmission(t);
					// 分岐1の最終実行時刻の更新
					t_last_1 = t;
				}

				// 分岐2： 上記以外のタイミングで常に実行
				else {
					//エンコーダによる各脚の関節角度情報
					all_angle_get(&enc);
					// 全ADポート出力値の取得
					getADall(AD_data);	// getAD()による逐次取得よりもgetADall()による一括取得の方が速い
					// FlexiForceによる接地情報取得
					all_FF_get(&ad, AD_data);
					// ポテンショメータによる接地情報取得
					all_POT_get(&ad, AD_data, t-t_last_2);
					// 接地判定
					touch_get(&ad, &load);
					// ジャイロセンサによる胴体傾き角度取得
					gyro_get(&ad, AD_data, t-t_last_2);
					// 加速度センサによる胴体傾き角度取得
					acc_get(&ad, AD_data, t-t_last_2);
					// 胴体傾き角度決定
					tilt_con(&ad);
					// 空気圧センサによる人工筋肉内気圧の取得
					air_press_get(&ad, AD_data);
					//電磁弁制御
					control_Muscles(t, unit);
					// サンプルデータの保存
//					data_get(k_sam, t, &sam, unit, &enc, &ad, &load); // データの格納
//					k_sam++;	// サンプルデータの要素数更新
					// 分岐3の最終実行時刻の更新
					t_last_2 = t;
				}
				// -------------------------------------------------------------------- //
				break;
			// ------ ステップ6（全筋肉弛緩3） ------ //
			case 6:
				muscle_reset();		// 全筋肉弛緩
				break;
		}
		// -------------------------------------------------------------------------------------------


		speed_treadmill_get(&enc, t-t_last);	// トレッドミル速度取得

//		if(t-t_last > 2.5e-3)
//			printf("%f\n", t-t_last);	// 計算周期表示
		t_last = t;

		t2 = (float)rdtsc();				// 関数処理直後の時刻
		t_proc = (float)((t2-t1)/tmu_frq/1.0e+6);	// 関数の処理にかかった時間を計算
		t_proc_sum += t_proc;				// 関数の合計処理時間を更新
		if(t_proc > t_proc_max) t_proc_max = t_proc;	// 最大処理時間の更新
		k_func++;					// 関数実行回数更新
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//------終了処理------//
	// PWMをOFF
	for(i=0; i<num_PWM; i++){
		setPWMduty(i, 0.0);	// duty比を全て0.0
		setPWMmode(i, OFF);	// PWMを全てOFF
	}
	// DIOをOFF
	for(i=0; i<num_DIO; i++)
		setDIO(i, OFF);		// DIOを全てOFF

	art_exit();		// リアルタイム制御終了
	closeHRP3sh4();		// デバイス使用終了

	//------ファイル書き込み------//
	printf("\t \t Writing......\n\n");		// ファイル書き込み中メッセージ表示
	file_output(k_sam, &sam);			// ファイルへデータを書き込む

	// 指定した関数の平均・最大処理時間の表示
	Calc_proc_time(k_func, t_proc_sum, t_proc_max);

	printf("\t \t All Program End.\n\n");	// プログラム終了メッセージ表示

	return 0;
}
