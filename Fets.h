/**
 * @file Fets.h
 * @brief FETモジュールを操作する主機能郡
 * @author Yuki HONMA @ projectR
 * @date 2019/10/18
 */

#ifndef FETS_H
#define FETS_H

#include <Arduino.h>

#define DEF_ID 0x90             /**< デフォルトID */

#define FUNC_DIGITAL_OUT 0x01   /**< 機能指定ビット デジタル出力 */
#define FUNC_PWM_OUT 0x02       /**< 機能指定ビット PWM出力 */
#define FUNC_SENSOR_RES 0x03    /**< 機能指定ビット センサ応答 */
#define FUNC_SENSOR_TRG 0x04    /**< 機能指定ビット センサトリガー */
#define FUNC_WAVE_SQU 0x05      /**< 機能指定ビット 矩形波出力 */
#define FUNC_WAVE_SINE 0x06     /**< 機能指定ビット 正弦波出力 */
#define FUNC_WAVE_TRI 0x07      /**< 機能指定ビット 三角波出力 */
#define FUNC_WAVE_SAW 0x08      /**< 機能指定ビット ノコギリ波出力 */
#define FUNC_WAVE_SAWINV 0x09   /**< 機能指定ビット 逆ノコギリ波出力 */

#define MODE_INIT 0             /**< クラスモード 初期化状態 */
#define MODE_CONFLICT -1        /**< クラスモード 指定の競合 */
#define MODE_MODULE 1           /**< クラスモード モジュール */
#define MODE_PORT 2             /**< クラスモード ポート */


/** 
 * @brief FETモジュール操作クラス
 *
 *
 * FETモジュールを操作する機能の抽象クラス @n
 * send() , recieve() が純粋仮想関数である @n
 *
 * このクラス内の公開メソッドが主機能すべてである @n
 *
 * @note すべての公開メソッドはそのまま通信を行うので割り込みなどには注意
 * @note すべての通信情報は 4byte である
 * @note シリアル通信115200[bps]で制御周期が10[ms]のとき， 36メソッド/回未満にするべきである
 *
 * @remarks 拡張クラスでデータ送受信を実装する必要がある
 *
 * @attention   このクラスは抽象クラスであり，インスタンス化できない@n
 *              派生クラスをインスタンス化する
 */
class Fets
{
    
public:

    /**
     * ポート番号 @n
     * メンバの列挙型なので外部から呼び出すにはスコープを解決しなければならない
     *
     * 例1) Fets::Out1 , Fets::In6
     */
    enum portNum{
        None = 0,   /**< 指定なし */

        Out1 = 1,   /**< 出力ポート1 */
        Out2,       /**< 出力ポート2 */
        Out3,       /**< 出力ポート3 */
        Out4,       /**< 出力ポート4 */
        Out5,       /**< 出力ポート5 */
        Out6,       /**< 出力ポート6 */
        Out7,       /**< 出力ポート7 Digital出力のみ */

        Dammy = 8,  /**< ダミー：使用不可 */

        In1 = 9,    /**< 入力ポート1 */
        In2,        /**< 出力ポート2 */
        In3,        /**< 出力ポート3 */
        In4,        /**< 出力ポート4 */
        In5,        /**< 出力ポート5 */
        In6,        /**< 出力ポート6 */
        In7,        /**< 出力ポート7 */
    };

    /**
     * 波出力の波形
     * メンバの列挙型なので外部から呼び出すにはスコープを解決しなければならない
     *
     * 例) Fets::Triangular , Fets::InvSawtooth
     */
    enum waveform{
        Square      = FUNC_WAVE_SQU,    /**< 矩形波 */
        Sine        = FUNC_WAVE_SINE,   /**< 正弦波 */
        Triangular  = FUNC_WAVE_TRI,    /**< 三角波 */
        Sawtooth    = FUNC_WAVE_SAW,    /**< ノコギリ波 */
        InvSawtooth = FUNC_WAVE_SAWINV, /**< 逆ノコギリ波 */
    };

    /**
     * コンストラクタ
     *
     * @note ポート番号を指定して実体化することで，そのクラス実体は指定したポート専用にできる
     * @note @n
     * @note ポート番号を指定せず実体化することで，そのクラス実体はモジュールとしてポートの指定は各メソッドで行う
     * @note ただしポート番号を指定して実体化しても，メソッドでポート指定されたときはそちらが優先される
     * @note 出力ポートを None，入力ポートのみを指定しての実体化をしようとするとモジュールとして実体化される
     *
     * @param _id           モジュールのID @n
     *                      デフォルトは @p 0x90
     * @param outputPort    使用する出力ポートの番号 Fets::Out1 ~ Fets::Out7 @n
     *                      指定しなくても良い
     * @param inputPort     使用する入力ポートの番号 Fets::In1 ~ Fets::In7 @n
     *                      指定しなくても良い
     *
     * @note                引数はすべてを指定しなくても良い @n
     *                      すべて指定しなければ、ID:0x90 となり，ポート指定を各メソッドで行うようになる @n
     *                      なおいずれの実体でもIDが同じなら getInputState() , getOutputState() で得られるポート情報は同じになる
     *
     * @attention   モジュールとしての実体を複数作ったり，ポート指定での実体を複数作ることは可能だが
     * @attention   モジュール実体とポート実体を混合させるのは不可能
     * @attention   モード干渉としてすべての公開メソッドで -1 を返す(はず)
     *
     */
    Fets(char _id = DEF_ID, portNum outputPort = None, portNum inputPort = None);


    /**
     * 出力
     *
     * digital出力を行う
     * @param duty          出力値 @p 0 or @p 1 (LOW or HIGH)
     * @param outputPort    出力ポートの番号 Fets::Out1 ~ Fets::Out7 @n
     *                      クラスをポート指定で実体化した場合必要ない
     *
     * @retval -1   出力ポート指定が不正
     * @retval -2   出力値指定が不正
     * @retval 0    正常
     */
    int write(int duty, portNum outputPort = None);

    /**
     * 出力
     *
     * PWM出力を行う
     * @param duty          出力値 @p 0.0 ~ @p 1.0
     * @param outputPort    出力ポートの番号 Fets::Out1 ~ Fets::Out6 @n
     *                      クラスをポート指定で実体化した場合必要ない
     *
     * @retval -1   出力ポート指定が不正
     * @retval -2   出力ポート指定が不正 PWM出力不可
     * @retval -3   出力値指定が不正 @p 0.0 未満
     * @retval -4   出力値指定が不正 @p 1.0 超過
     * @retval 0    正常
     * 
     * @remarks Fets::Out7 はPWM出力ができない
     *
     * @overload
     */
    int write(double duty, portNum outputPort = None);

    /**
     * センサ応答を設定する
     *
     * センサの入力に応じて出力する @n
     * センサの入力が変化した場合，出力も変化する
     * @param actInput      入力値 @p 0 or @p 1 (LOW or HIGH) @n
     *                      指定された入力状態になった時動作する
     * @param actOutput     出力値 @p 0 or @p 1 (LOW or HIGH) @n
     *                      動作すると指定された出力をする
     * @param outputPort    電磁弁出力ポートの番号 Fets::Out1 ~ Fets::Out7 @n
     *                      クラスをポート指定で実体化した場合必要ない
     * @param inputPort     センサ入力ポートの番号 Fets::In1 ~ Fets::In7 @n
     *                      クラスをポート指定で実体化した場合必要ない
     *
     * @retval -1   出力ポート指定が不正
     * @retval -2   入力ポート指定が不正
     * @retval 1    動作完了： actOutput で指定された出力をしている @n
     *              ただし recvData() または getOutputState() または getInputState() が実行されていなければ動作完了は返さない
     * @retval 0    正常
     */
    int sensorResponce(uint8_t actInput, uint8_t actOutput, portNum outputPort = None, portNum inputPort = None);

    /**
     * センサ応答を設定する
     *
     * センサの入力に応じて出力する @n
     * 一度動作した後，センサの入力が変化しても，出力は変化しない
     * @param actInput      入力値 @p 0 or @p 1 (LOW or HIGH) @n
     *                      指定された入力状態になった時動作する
     * @param actOutput     出力値 @p 0 or @p 1 (LOW or HIGH) @n
     *                      動作すると指定された出力をする
     * @param outputPort    出力ポートの番号 Fets::Out1 ~ Fets::Out7 @n
     *                      クラスをポート指定で実体化した場合必要ない
     * @param inputPort     入力ポートの番号 Fets::In1 ~ Fets::In7 @n
     *                      クラスをポート指定で実体化した場合必要ない
     *
     * @retval -1   出力ポート指定が不正
     * @retval -2   入力ポート指定が不正
     * @retval 1    動作完了： actOutput で指定された出力をしている @n
     *              ただし recvData() または getOutputState() または getInputState() が実行されていなければ動作完了は返さない
     * @retval 0    正常
     *
     * @note    この機能では出力の初期化を行わない
     * @note    この機能が呼び出された時，すでに動作時指定出力になっていた場合動作完了となる
     */
    int sensorTrigger(uint8_t actInput, uint8_t actOutput, portNum outputPort = None, portNum inputPort = None);

    /**
     * 出力
     *
     * 特定の波形で出力する
     * @param form          波形を指定する @p enum
     * @param outputPort    出力ポートの番号 Fets::Out1 ~ Fets::Out6 @n
     *                      クラスをポート指定で実体化した場合必要ない
     * @param period        周期[ms] @p 100 ~ @p 10000
     *
     * @retval -1   出力ポート指定が不正
     * @retval -2   出力ポート指定が不正 PWM出力不可
     * @retval -3   周期指定が不正 @p 100 未満
     * @retval -4   周期指定が不正 @p 10000 超過
     * @retval 0    正常
     */
    int writeWave(waveform form, int period, portNum outputPort = None);


    /**
     * 出力状態を取得する
     *
     * @param outputPort    状態を読みたい出力ポートの番号 Fets::Out1 ~ Fets::Out7 @n
     *                      Fets::None を指定するか，指定がなければ7ビットにすべての出力ポートの情報を格納した値を返す
     *
     * @retval 1or0                     ポート指定時：指定されたポートの出力状態 @n
     * @retval 0b00000000~0b01111111    ポート非指定時：すべてのポートの出力状態 右から出力ポート1
     *
     * @note    この関数内部で recvData() が実行されている
     * @attention recvData() のattentionに留意
     */
    int getOutputState(portNum outputPort = None);

    /**
     * 入力状態を取得する
     *
     * @param inputPort    状態を読みたい入力ポートの番号 Fets::In1 ~ Fets::In7 @n
     *                      Fets::None を指定するか，指定しなければ7ビットにすべての入力ポートの情報を格納した値を返す
     *
     * @retval              1or0 ポート指定時：指定されたポートの入力状態 @n
     * @retval              0b00000000~0b01111111 ポート非指定時：すべてのポートの入力状態 右から入力ポート1
     *
     * @note    この関数内部で recvData() が実行されている
     * @attention recvData() のattentionに留意
     */
    int getInputState(portNum inputPort = None);


    /**
     * 受信データから情報を取り出し，メンバ変数に格納する
     * @return 読み込んだデータ数
     * @attention 同じモジュールの情報を取得する場合，1つのクラス実体だけで，このメソッドを呼び出すこと．
     * @attention 実体ごとに受信バッファがあるため，正しく受信できなくなってしまう．
     */
    int recvData();


protected:

    /**
     * データを送信する関数 @n
     * 呼び出しはメンバ関数が行う
     *
     * 純粋仮想関数であり，実装は拡張クラスが行う
     *
     * @param data 出力データ
     */
    virtual void send(char data) = 0;

    /**
     * 受信データを返す関数 @n
     * 呼び出しはメンバ関数が行う
     *
     * 純粋仮想関数であり，実装は拡張クラスが行う
     * 
     * @retval -1 新規データなし @n
     * @retval 0~0xFF 受信したデータ 
     */
    virtual int recieve() = 0;

    /**
     * 送信用データを作成し send() に送る @n
     * メンバ以外で呼び出しはしない
     *
     * @param funcBit       機能指定ビット
     * @param outputPort    出力ポートの番号
     * @param parameter     送信パラメータ
     *
     * @retval  0 正常
     * @retval  -1 ポート指定が不正
     */
    int sendData(uint8_t funcBit, portNum outputPort, uint8_t parameter);

private:

    /**
     * 指定した出力ポートが適正か確認する
     *
     * @param outputPort    出力ポートの番号
     *
     * @retval  Fets::Out1~Fets::Out7 出力ポートの番号
     * @retval  Fets::None ポート指定が不正
     */
    portNum opCheck(portNum outputPort);

    /**
     * 指定した入力ポートが適正か確認する
     *
     * @param outputPort    入力ポートの番号
     *
     * @retval  Fets::In1~Fets::In7 入力ポートの番号
     * @retval  Fets::None ポート指定が不正
     */
    portNum ipCheck(portNum inputPort);

    /**
     * クラスをポート指定で実体化したときの出力番号
     */
    portNum opNum;

    /**
     * クラスをポート指定で実体化したときのセンサ入力番号
     */
    portNum ipNum;


    uint8_t inputState;
    uint8_t outputState;

    /**
     * 受信情報の配列
     */
    char dataBuff[4];

    /**
     * クラスがモジュールとして実体化されたか，ピン指定で実体化されたかの状態を格納 @n
     * 複数のクラスで統一させるため，静的メンバ変数とする
     */
    static char mode;

    /**
     * スレーブモジュールのID
     */
    char id;
};



#endif