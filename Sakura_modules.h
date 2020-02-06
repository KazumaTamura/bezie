/**
 * @file    Sakura_modules.h
 * @brief   GR-SAKURA でモジュールを使用するための主機能の通信拡張 @n
 *          Fets の拡張 S_Fets @n
 *          UnderBody の拡張 S_UnderBody
 * @author  Yuki HONMA @ ProjectR
 * @date    2019/10/17
 */


#ifndef SAKURA_MODULES_H
#define SAKURA_MODULES_H

#include <Arduino.h>


#define USE_FET 1       /**< FETモジュールライブラリの使用の有無を選択する． 使用時は1，不使用時は0にする． */
#define USE_UNDERBODY 1 /**< UnderBodyモジュールライブラリの使用の有無を選択する． 使用時は1，不使用時は0にする */



#if USE_FET == 1

#include "Fets.h"


/**
 * 使用例
 *
 *  @code 
 *
 *  #include <Arduino.h>
 *  #include "Sakura_modules.h"
 *
 *  // モジュールとしてインスタンス化
 *  // インスタンス化するのは派生クラスの方
 *  S_Fets Module_S(&Serial1);
 *
 *
 *  void setup(){
 *
 *      Serial.begin(230400);
 *
 *      // シリアル開始． Serial1.begin(115200); でも可能
 *      Module_S.begin(115200);
 *  }
 *
 *  void loop(){
 *      delay(10);
 *
 *      // Digital出力
 *      // 派生クラスは基底クラスのメンバを継承してるのでそのまま使う
 *      Module_S.write(1, Fets::Out1);
 *
 *      // PWM出力
 *      Module_S.write(0.4, Fets::Out2);
 *
 *      // センサ応答設定
 *      Module_S.sensorResponce(0, 0, Fets::Out3, Fets::In1);
 *
 *      // センサ応答設定
 *      Module_S.sensorResponce(0, 1, Fets::Out4, Fets::In1);
 *
 *      // 三角波出力
 *      Module_S.writeWave(Fets::Triangular, 2000, Fets::Out5);
 *
 *      // センサ情報読み取り
 *      if(Module_S.getInputState(Fets::In3)){
 *
 *          // Digital出力
 *          Module_S.write(1, Fets::Out7);
 *
 *          // センサトリガー設定
 *          Module_S.sensorTrigger(0, 1, Fets::Out6, Fets::In2);
 *      }
 *      else{
 *
 *          // Digital出力
 *          Module_S.write(0, Fets::Out6);
 *
 *          // センサトリガー設定
 *          Module_S.sensorTrigger(0, 0, Fets::Out7, Fets::In2);
 *      }
 *  }
 *  @endcode
 */

/**
 * 使用例2
 * @code
 * 
 *  #include <Arduino.h>
 *
 *  #include "Sakura_modules.h"
 *
 *  #define FET_MOD_ID 0x90
 *
 *  // ポートごとに実体化
 *  S_Fets Sorenoid1(&Serial1, FET_MOD_ID, Fets::Out1, Fets::In1);
 *  S_Fets Sorenoid2(&Serial1, FET_MOD_ID, Fets::Out2, Fets::In1);
 *  S_Fets Sorenoid3(&Serial1, FET_MOD_ID, Fets::Out3, Fets::In2);
 *  S_Fets Sorenoid4(&Serial1, FET_MOD_ID, Fets::Out4, Fets::In3);
 *  S_Fets Lamp(&Serial1, FET_MOD_ID, Fets::Out5);
 *  S_Fets RedLamp(&Serial1, FET_MOD_ID, Fets::Out6);
 *  S_Fets BlueLamp(&Serial1, FET_MOD_ID, Fets::Out7);
 * 
 *  void setup(){
 *      Serial.begin(230400);
 * 
 *      // 通信開始
 *      Serial1.begin(115200);
 *  }
 *  
 *  void loop(){
 *  
 *      Serial.println(Sorenoid1.sensorResponce(0, 0));
 *      Sorenoid2.sensorResponce(0, 1);
 *  
 *      // 情報読み取りはどの実体でもよいが，別々の実体で何回も呼び出さないほうが良い
 *      if(Sorenoid3.getInputState(Fets::In1)){
 *          Sorenoid3.sensorTrigger(0, 1);
 *          Sorenoid4.sensorTrigger(0, 1);
 *      }
 *      else{
 *          Sorenoid3.write(0);
 *          Sorenoid4.write(0);
 *      }
 *  
 *      Lamp.writeWave(Fets::InvSawtooth, 5000);
 *      RedLamp.writeWave(Fets::Square, 500);
 *      BlueLamp.write(1);
 *  
 *    delay(10);
 *  }
 * 
 * @endcode
 */


/**
 * @brief FETモジュール操作機能の GR-SAKURA 実装用クラス
 *
 *
 * Fets クラスを継承した拡張クラス @n
 * Fets クラスのパブリックメンバをそのまま呼び出せる
 *
 * @note    GR-SAKURA のシリアル通信 HardwareSerial を使用して send() , recieve() メソッドをオーバーライドしている
 *
 */
class S_Fets : public Fets
{
public:

    /**
     * コンストラクタ
     *
     *
     * 引数はシリアル通信のポインタと基底クラスのコンストラクタ Fets::Fets() に渡す情報
     * 
     * @param _comm         モジュールとの通信に使用するハードウェアシリアルのポインタ
     * @param _id           モジュールのID 基底クラスにそのまま渡す @n
     * @param outputPort    使用する出力ポートの番号 基底クラスにそのまま渡す @n
     * @param inputPort     使用する入力ポートの番号 基底クラスにそのまま渡す @n
     *
     * @remarks             _id , outputPort , inputPort について詳細は基底クラス Fets::Fets() に記述してある
     */
    S_Fets(HardwareSerial *_comm, char _id = DEF_ID, Fets::portNum outputPort = Fets::None, Fets::portNum inputPort = Fets::None);

    /**
     * シリアル通信を開始する
     *
     * Serial_.begin() と同義
     * 
     * @param baudrate      ボーレート
     *
     * @note        シリアルの実体でbeginしてもよい
     * @attention   マルチスレーブ接続のため複数のモジュールで同一のシリアル通信バスを使用する場合，シリアルの実体でbeginしたほうがよい
     */
    void begin(int baudrate = 115200);

protected:

    /**
     * シリアル通信での送信メソッド @n
     * 外部呼び出しはされない
     *
     * @param data  送信するデータ(1byte)
     */
    void send(char data); //override

    /**
     * シリアル通信での受信メソッド @n
     * 外部呼び出しはされない
     *
     * @return @p -1 新規データなし
     * @return @p 0x00 ~ @p 0xFF 受信したデータ
     */
    int recieve(); //override

private:

    HardwareSerial *comm;
};

#endif

#if USE_UNDERBODY == 1
#include "UnderBody.h"


/**
 * 使用例 アナログ電圧に応じてvX, vY, omegaを与える．
 * 移動平均している
 *
 * @code
 *  #include <arduino.h>
 *
 *  #include "Sakura_modules.h"
 *
 *  #define PIN_CONT_X A0
 *  #define PIN_CONT_Y A1
 *  #define PIN_CONT_T A2
 *
 *  #define CONT_SPEED 1.0
 *  #define CONT_ANG 3.14
 *
 *  #define BUFF_SMOOTH 50
 *
 *  // インスタンス化
 *  S_UnderBody Omni4(&Serial1);
 *
 *  void setup(){
 *
 *      pinMode(PIN_LED0, OUTPUT);
 *      analogReference(RAW12BIT);
 *
 *      digitalWrite(PIN_LED0, HIGH);
 *      
 *      // 通信開始
 *      Omni4.begin(115200);
 *  }
 *
 *  void loop(){
 *
 *      static double vX[BUFF_SMOOTH + 1] = {}, vY[BUFF_SMOOTH + 1] = {}, omega[BUFF_SMOOTH + 1] = {};
 *      double sumVX = 0, sumVY = 0, sumOmega = 0;
 *
 *      // アナログ値を速度に変換
 *      vX[0] = ((double)analogRead(PIN_CONT_X) - 2048.0)*CONT_SPEED / 4096.0;
 *      vY[0] = ((double)analogRead(PIN_CONT_Y) - 2048.0)*CONT_SPEED / 4096.0;
 *      omega[0] = ((double)analogRead(PIN_CONT_T) - 2048.0)*CONT_ANG / 4096.0;
 *
 *      //移動平均
 *      for(int i=BUFF_SMOOTH; i>0; i--){
 *          vX[i] = vX[i-1];
 *          vY[i] = vY[i-1];
 *          omega[i] = omega[i-1];
 *
 *          sumVX += vX[i];
 *          sumVY += vY[i];
 *          sumOmega += omega[i];
 *      }
 *
 *      //送信
 *      Omni4.moveXY(sumVX/(double)BUFF_SMOOTH, sumVY/(double)BUFF_SMOOTH, sumOmega/ (double)BUFF_SMOOTH);
 *
 *      delay(10);
 *  }
 * @endcode
 */

/**
 * @brief UnderBodyモジュール操作機能の GR-SAKURA 実装用クラス
 *
 *
 * UnderBody クラスを継承した拡張クラス @n
 * UnderBody クラスのパブリックメンバをそのまま呼び出せる
 *
 * GR-SAKURA のシリアル通信 HardwareSerial を使用して send() メソッドをオーバーライドしている
 */
class S_UnderBody : public UnderBody
{
public:

    /**
     * コンストラクタ
     *
     * @param _comm モジュールとの通信に使用するハードウェアシリアルのポインタ
     */
    S_UnderBody(HardwareSerial *_comm);

    /**
     * シリアル通信を開始する．
     *
     * @param baudrate ボーレート
     *
     * @note        シリアルの実体でbeginしてもよい
     * @attention   マルチスレーブ接続のため複数のモジュールで同一のシリアル通信バスを使用する場合，シリアルの実体でbeginしたほうがよい
     */
    void begin(int baudrate = 115200);

protected:

    /**
     * シリアル通信での送信メソッド @n
     * 外部呼び出しはされない
     *
     * @param data 送信するデータ(1byte)
     */
    void send(char data); //override

private:
    HardwareSerial *comm;

};
#endif

#endif