/**********************************************************************/
/* モータ・スイッチ入出力 (前後モータPWM, ステアPWM, DIPSW/プッシュSW)
/**********************************************************************/


/************************************************************************/
/* マイコンボード上のディップスイッチ値読み込み                         */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0～15                                              */
/************************************************************************/
unsigned char dipsw_get(void)
{
    return ((!R_PORT3->PIDR_b.PIDR6) << 1) | ((!R_PORT3->PIDR_b.PIDR7) << 0);
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のプッシュスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0:OFF 1:ON                                         */
/************************************************************************/
unsigned char pushsw_get(void)
{
    return !(R_PORT0->PIDR_b.PIDR3);
}

/************************************************************************/
/* 後輪の速度制御                                                       */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_r(int accele_l, int accele_r)
{
    // モータ停止モード(dipsw_get() & 0x01)
    if (!(dipsw_get() & 0x01))
    {
        motor2_r(accele_l * -1, accele_r);
    }
    else if (pattern >= 11)
    {
        motor2_r(-10, 10);
    }
}

/************************************************************************/
/* 後輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_r(int accele_l, int accele_r)
{


    // --- 左モーターの制限（配線が逆なので、前進がマイナス、ブレーキがプラス） ---
    if (accele_l < -Motor_Max_PWM)
    {
        accele_l = -Motor_Max_PWM; // 前進の制限
    }
    if (accele_l > 100)
    {
        accele_l = 100; // ブレーキは100%まで許可
    }

    // --- 右モーターの制限（通常通り、前進がプラス、ブレーキがマイナス） ---
    if (accele_r > Motor_Max_PWM)
    {
        accele_r = Motor_Max_PWM; // 前進の制限
    }
    if (accele_r < -100)
    {
        accele_r = -100; // ブレーキは-100%まで許可
    }

    motor_buff_Rl = accele_l * -1;
    motor_buff_Rr = accele_r;

    // 左モータ制御
    if (accele_l > 0)
    {
        RL_A = HIGH;
        RL_B = LOW;
        MOTOR_RL_PWM = (long)(MOTOR_RL_PWM_CYCLE + 1) * accele_l / 100;
    }
    else if (accele_l < 0)
    {
        RL_A = LOW;
        RL_B = HIGH;
        MOTOR_RL_PWM = (long)(MOTOR_RL_PWM_CYCLE + 1) * (-accele_l) / 100;
    }

    else
    {
        RL_A = LOW;
        RL_B = LOW;
        MOTOR_RL_PWM = (long)(MOTOR_RL_PWM_CYCLE + 1) * (-accele_l) / 100;
    }

    // 右モータ制御
    if (accele_r > 0)
    {
        RR_A = HIGH;
        RR_B = LOW;
        MOTOR_RR_PWM = (long)(MOTOR_RR_PWM_CYCLE + 1) * accele_r / 100;
    }
    else if (accele_r < 0)
    {
        RR_A = LOW;
        RR_B = HIGH;
        MOTOR_RR_PWM = (long)(MOTOR_RR_PWM_CYCLE + 1) * (-accele_r) / 100;
    }

    else
    {
        RR_A = LOW;
        RR_B = LOW;
        MOTOR_RR_PWM = (long)(MOTOR_RR_PWM_CYCLE + 1) * (-accele_r) / 100;
    }
}

/************************************************************************/
/* 前輪の速度制御                                                       */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_f(int accele_l, int accele_r)
{
    // モータ停止モード
    if (!(dipsw_get() & 0x01))
    {
        motor2_f(accele_l * -1, accele_r);
    }
    else if (pattern >= 11)
    {
        motor2_f(-10, 10);
    }
}

/************************************************************************/
/* 前輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_f(int accele_l, int accele_r)
{




    // --- 左モーターの制限（配線が逆なので、前進がマイナス、ブレーキがプラス） ---
    if (accele_l < -Motor_Max_PWM)
    {
        accele_l = -Motor_Max_PWM; // 前進の制限
    }
    if (accele_l > 100)
    {
        accele_l = 100; // ブレーキは100%まで許可
    }

    // --- 右モーターの制限（通常通り、前進がプラス、ブレーキがマイナス） ---
    if (accele_r > Motor_Max_PWM)
    {
        accele_r = Motor_Max_PWM; // 前進の制限
    }
    if (accele_r < -100)
    {
        accele_r = -100; // ブレーキは-100%まで許可
    }

    motor_buff_Fl = accele_l * -1;
    motor_buff_Fr = accele_r;


    // 左モータ制御
    if (accele_l > 0)
    {
        FL_A = HIGH;
        FL_B = LOW;
        MOTOR_FL_PWM = (long)(MOTOR_FL_PWM_CYCLE + 1) * accele_l / 100;
    }
    else if (accele_l < 0)
    {
        FL_A = LOW;
        FL_B = HIGH;
        MOTOR_FL_PWM = (long)(MOTOR_FL_PWM_CYCLE + 1) * (-accele_l) / 100;
    }
    else
    {
        FL_A = LOW;
        FL_B = LOW;
        MOTOR_FL_PWM = (long)(MOTOR_FL_PWM_CYCLE + 1) * (-accele_l) / 100;
    }

    // 右モータ制御
    if (accele_r > 0)
    {
        FR_A = HIGH;
        FR_B = LOW;
        MOTOR_FR_PWM = (long)(MOTOR_FR_PWM_CYCLE + 1) * accele_r / 100;
    }
    else if (accele_r < 0)
    {
        FR_A = LOW;
        FR_B = HIGH;
        MOTOR_FR_PWM = (long)(MOTOR_FR_PWM_CYCLE + 1) * (-accele_r) / 100;
    }
    else
    {
        FR_A = LOW;
        FR_B = LOW;
        MOTOR_FR_PWM = (long)(MOTOR_FR_PWM_CYCLE + 1) * (-accele_r) / 100;
    }
}

/************************************************************************/
/* サーボモータ制御                                                     */
/* 引数　 サーボモータPWM：-100～100                                    */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void servoPwmOut(int pwm)
{
    motor_buff_stare = pwm;

    // モータ制御
    // 限界設定
    if (abs(getServoAngle()) < 130)
    {
        if (pwm >= 0)
        {
            ST_A = HIGH;
            ST_B = LOW;
            MOTOR_ST_PWM = (long)(MOTOR_ST_PWM_CYCLE + 1) * pwm / 100;
        }
        else
        {
            ST_A = LOW;
            ST_B = HIGH;
            MOTOR_ST_PWM = (long)(MOTOR_ST_PWM_CYCLE + 1) * (-pwm) / 100;
        }
    }
    else
    {
        MOTOR_ST_PWM = 0;
    }
}

/**********************************************************************/
/**
 *	モータ関係 初期化.
 */
void initMotor(void)
{
    /* RRモータ */
    // defineで定義
    MOTOR_INIT_RR;

    /* RLモータ */
    // defineで定義
    MOTOR_INIT_RL;

    /* FRモータ */
    // defineで定義
    MOTOR_INIT_FR;

    /* FLモータ */
    // defineで定義
    MOTOR_INIT_FL;

    /* STモータ */
    // defineで定義
    MOTOR_INIT_ST;
}
