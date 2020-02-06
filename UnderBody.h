/**
 * @file UnderBody.h
 * @brief 足回りモジュールを操作する主機能郡
 * @author Yuki HONMA @ ProjectR
 * @date 2019/10/25
 */

#ifndef UNDER_BODY_H
#define UNDER_BODY_H


#include <Arduino.h>

#define MOVE_RECT   0xFF
#define MOVE_POLAR  0xFE
#define MOVE_STOP   0xF0

#define MAX_VELO    8000
#define MAX_OMEGA   500


/**
 * @brief 足回りモジュール操作クラス
 *
 *
 * 全方向移動機構を想定した足回りモジュールを操作するための抽象クラス @n
 * send() が純粋仮想関数である
 *
 * @note すべての公開メソッドはそのまま通信を行うので割り込みなどには注意
 * @note すべての通信情報は 8byte である
 *
 * @remarks 拡張クラスでデータ送受信を実装する必要がある
 *
 * @attention   このクラスは抽象クラスであり，インスタンス化できない @n
 *              派生クラスをインスタンス化する
 */
class UnderBody
{
public:

    /**
     * コンストラクタ
     * 
     */
    UnderBody();

    /**
     * 直交座標系として移動速度を与える．
     * 座標系はロボットに固定されている．
     *
     * @param vX X方向の移動速度 @p -8000 ~ @p 8000 [mm/s]
     * @param vY Y方向の移動速度 @p -8000 ~ @p 8000 [mm/s]
     * @param omega 機体の旋回速度 @p -500 ~ @p 500 [deg/s]
     *
     * @retval 0 正常
     * @retval -1 vX指定が不正 最大値超過
     * @retval -2 vX指定が不正 最小値未満
     * @retval -3 vY指定が不正 最大値超過
     * @retval -4 vY指定が不正 最小値未満
     * @retval -5 omega指定が不正 最大値超過
     * @retval -6 omega指定が不正 最小値未満
     *
     * @note 引数はすべてint型である
     */
    int moveXY(int vX, int vY, int omega);

    
    /**
     * 直交座標系として移動速度を与える．
     * 座標系はロボットに固定されている．
     *
     * @param vX X方向の移動速度 @p -8.0 ~ @p 8.0 [m/s]
     * @param vY Y方向の移動速度 @p -8.0 ~ @p 8.0 [m/s]
     * @param omega 機体の旋回速度 @p -8.72 ~ @p 8.72 [rad/s]
     *
     * @retval 0 正常
     * @retval -1 vX指定が不正 最大値超過
     * @retval -2 vX指定が不正 最小値未満
     * @retval -3 vY指定が不正 最大値超過
     * @retval -4 vY指定が不正 最小値未満
     * @retval -5 omega指定が不正 最大値超過
     * @retval -6 omega指定が不正 最小値未満
     *
     * @note 引数はすべてdouble型である
     *
     * @overload
     */
    int moveXY(double vX, double vY, double omega);

    /**
     * 極座標系として移動速度を与える．
     * 座標系はロボットに固定されている．
     *
     * @param spd 機体の移動速さ @p -8000 ~ @p 8000 [mm/s]
     * @param dir 機体の移動方向 [deg]
     * @param omega 機体の旋回速度 @p -500 ~ @p 500 [deg/s]
     *
     * @retval 0 正常
     * @retval -1 spd指定が不正 最大値超過
     * @retval -2 spd指定が不正 最小値未満
     * @retval -3 omega指定が不正 最大値超過
     * @retval -4 omega指定が不正 最小値未満
     *
     * @note 引数はすべてint型である
     */
    int movePolar(int spd, int dir, int omega);

    /**
     * 極座標系として移動速度を与える．
     * 座標系はロボットに固定されている．
     *
     * @param spd 機体の移動速さ @p -8.0 ~ @p 8.0 [m/s]
     * @param dir 機体の移動方向 [rad]
     * @param omega 機体の旋回速度 @p -8.72 ~ @p 8.72 [rad/s]
     *
     * @retval 0 正常
     * @retval -1 spd指定が不正 最大値超過
     * @retval -2 spd指定が不正 最小値未満
     * @retval -3 omega指定が不正 最大値超過
     * @retval -4 omega指定が不正 最小値未満
     *
     * @note 引数はすべてdouble型である
     *
     * @overload
     */
    int movePolar(double spd, double dir, double omega);

    /**
     * 動作を停止する．
     */
    void stop();

protected:

    /**
     * 送信用データを作成し send() に送る
     * 外部呼び出しはされない
     *
     * @param param1    送信パラメータ1
     * @param param2    送信パラメータ2
     * @param param3    送信パラメータ3
     * @param mode      モード @p MOVE_RECT,MOVE_POLAR,MOVE_STOP
     */
    void sendData(int param1, int param2, int param3, uint8_t mode);

    /**
     * データを送信する関数 @n
     * 外部呼び出しはされない
     *
     * 純粋仮想関数であり，実装は拡張クラスが行う
     *
     * @param data 出力データ
     */
    virtual void send(char data) = 0;

private:

};

#endif