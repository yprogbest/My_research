#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import smbus                #I2C通信するためのモジュールsmbusをインポートする
import time                 #sleepするためにtimeモジュールをインポートする

"""メイン関数"""
if __name__ == '__main__':
    bus = smbus.SMBus(1)    ##I2C通信するためのモジュールsmbusのインスタンスを作成
    adress = 0x04           #arduinoのサンプルプログラムで設定したI2Cチャンネル

    try:
        while True:
            #Arduinoへ文字『R』を送る、ordはアスキーコードを取得
            bus.write_byte(adress, ord('I am yuki'))

            #Arduinoからのメッセージを取得し表示する、chrはアスキーコードを文字へ変換
            msg = chr(bus.read_byte(adress))
            print(msg)

            #0.5sスリープする
            time.sleep(1)

    except KeyboardInterrupt  :         #Ctl+Cが押されたらループを終了
        print("\nCtl+C")
    except Exception as e:
        print(str(e))
    finally:
        print("\nexit program")