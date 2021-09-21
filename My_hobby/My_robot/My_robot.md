# 自律走行ロボット（自作）
![0C4ED5E6-5970-443A-A038-4FA338512422](https://user-images.githubusercontent.com/82224433/133868393-978f8b44-b621-494f-ab46-6bf3143bad08.jpg)


## 開発環境
- Ubuntu 20.04 LTS
- ROS noetic
- ydLiDAR
- iPhone7(Camera)→droidcamを使用 ROS対応
- Arduino
- ラズパイ4B+

## 自律走行
- [Google Cartographer](https://google-cartographer.readthedocs.io/en/latest/)
- [Navigation Stack(move_base, amcl)](https://qiita.com/MoriKen/items/0b75ab291ab0d95c37c2)

## Cartographerによる地図作成結果
![03DAB047-AD4F-42A5-A0C2-143BC3C54EF5](https://user-images.githubusercontent.com/82224433/133687418-53c38e20-08d2-441c-93cc-af1db73ce4d7.jpg)

## move_baseによる自律走行結果（2021/9/15日現在　良好な結果が得られていない）
![ダウンロード](https://user-images.githubusercontent.com/82224433/133687569-9de053a7-af62-4e49-bc87-7bb1f85ca9ef.gif)

## 2D LiDARとカメラ（iPhone）のキャリブレーション結果（2021/9/21現在）
![0A3A9831-E2D3-4E2D-8409-C412A52AD1D3](https://user-images.githubusercontent.com/82224433/134201263-c34e967c-4f24-496c-8d2f-41b52b0c6b81.jpg)<br>
![14241707-3AF6-4E09-857A-81D627BAD2CE](https://user-images.githubusercontent.com/82224433/134201357-0f0bd674-4327-4268-82b6-60d3ebbc1295.jpg)
- [キャリブレーションで用いた手法(github)](https://github.com/ehong-tl/camera_2d_lidar_calibration)
- githubからクローンしたプログラムファイルを実行しようとすると，エラーが表示された．そのため，自身でプログラムを改編したところ，誤った所にLiDARの点群が投影される箇所が存在することが分かった．今後は，再度プログラムを確認するとともに，Pythonでのコードの書き方を理解する必要がある．
