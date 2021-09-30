# My research

## 研究紹介
### ２０２1後期（修士1年）

#### 3. 〇〇(/)
- [Slide]
- [Report]

##### 要旨


##### 実験結果


#### 2. 〇〇(/)
- [Slide]
- [Report]

##### 要旨


##### 実験結果


#### 1. Mask R-CNNとLiDAR点群を用いた物体追跡システムの開発(10/1 夏休みの活動)
- [Slide](https://github.com/yuki-research/My_research/files/7259945/10_1_P_Masuda.pdf)
- スライド構成図
![568EFD73-A36E-40F3-A6B4-1889553F12EE](https://user-images.githubusercontent.com/82224433/135467042-93cf71e1-ebf6-4cb5-83b3-5c10afa9c1f8.png)

##### 実験結果
Mask R-CNNによって認識された物体領域内のLiDAR点群を抽出することで，物体位置の計測を試みた．LiDARの点群を画像に投影する際のずれや点群数が少ないことから，良好に物体の位置を計測吸うことが出来ないという結果が得られた．今後は，今回導入したLiDARよりも点群を多く取得することが出来るLivox LiDARを使うことで，物体位置の計測ならびに追跡を行っていく．


### ２０２1前期（修士1年）

#### 2. インスタンスセグメンテーションを用いた作業空間内物体認識システムの開発(6/11)
- [Slide](https://github.com/yuki-research/My_research/files/6641805/0611_P_Masuda.pdf)
- [Report](https://github.com/yuki-research/My_research/files/6641804/0611_W_Masuda.pdf)

##### 要旨
卒業研究では，広範囲の農作業空間内における複数の物体の動きを追跡するシステムを開発した．
このシステムでは，広範囲の空間を把握することのできる LiDAR と深層学習モデル YOLOv3 によ
るカメラベースの物体認識を組み合わせ，物体の位置計測ならびに追跡を行った．しかし，YOLOv3
によって生成された物体領域内には，物体以外に背景も含まれるため，良好な追跡結果が得られない
という課題があった．そこで，本研究では，YOLOv3 に替わり，画像のピクセル単位で物体認識を
行うことのできる，インスタンスセグメンテーションを導入した．実験では，YOLOv3 とインスタ
ンスセグメンテーションの認識結果に対するそれぞれの背景の割合を比較し，YOLOv3 では約 58 %
の背景が含まれるのに対して，インスタンスセグメンテーションでは，約 5.0 ％という結果が得ら
れた．今回の研究を通して，物体追跡システムの開発において，インスタンスセグメンテーションを
導入することに有用性があることが証明された．今後は，LiDAR とインスタンスセグメンテーショ
ンを組み合わせることで，3 次元空間内での物体の位置計測ならびに追跡を行っていく．


##### Mask R-CNN
Mask R-CNNは，Facebook AI Research の
kaiming He, Ross Girshick らにより提案され
た，一般物体検出（Generic Object Detection）
とインスタンスセグメンテーションを同時に
行うマルチタスクの手法である．まず，画像中の
物体らしき領域とその領域が表すクラスを検出
する．物体らしき領域は，画像を特定の領域に区
切り，それらを逐一評価することで得られる．ク
ラスは，学習で利用したデータセットに含まれ
るクラスそれぞれについて，物体らしさを指す
確率で得られる．その中で，最も確率の高いクラ
スのみを採用する．そして，物体検出結果として
得られた領域についてのみセグメンテーション
を行う．


##### YOLOv3との比較
卒業研究では，YOLOv3 によって生成された
物体領域に，追跡対象とする物体だけではなく
背景も含まれることで，頑健な物体位置の計測
を行うことができないという問題点があった．
そこで，本実験では，あらかじめ追跡対象とする
物体のマスク画像を作成し，YOLOv3 によって
生成された物体領域内の背景の割合とインスタ
ンスセグメンテーションによって認識された領
域内の背景の割合を比較することで，インスタ
ンスセグメンテーションの有用性を検証する．



##### 実験結果

- Mask R-CNNの認識結果<br>
![image](https://user-images.githubusercontent.com/82224433/121768599-ec4f8000-cb99-11eb-99c8-51b4904d73ab.png)

- YOLOv3との比較結果<br>
![image](https://user-images.githubusercontent.com/82224433/121768635-1e60e200-cb9a-11eb-825a-c72640d6a7e2.png)<br>
![image](https://user-images.githubusercontent.com/82224433/121768672-59631580-cb9a-11eb-9aae-70a8ab4dc792.png)


##### Mask R-CNNの問題点
- バウンディングボックスを生成してから，その範囲内の画素を認識する，2ステップの工程を踏むことで，物体の認識を行っている．そのため，処理速度が遅いという問題点がある．
- 今後は，処理速度がMask RCNNよりも高速かつ，ROS対応のYOLACTについて調べていく！<br>![image](https://user-images.githubusercontent.com/82224433/121992328-63745680-cddc-11eb-86d9-8f23b4251362.png)




#### 1. 卒業研究と今後の取り組み(4/9)
- [Slide](https://github.com/yuki-research/My_research/files/6286897/0409_P_Masuda.pdf)


----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
## 研究紹介
### ２０２０後期（4年）
#### 8. 学会発表会(3/2)
- [Slide](https://github.com/yuki-research/My_research/files/6286909/P_Masuda_.pdf)
- [Report](https://github.com/yuki-research/My_research/files/6286914/517359_masuda_.pdf)

##### はじめに
近年，スマート農業の旗印の下で，大規模稲作向けのロボットや収穫ロボットの開発が進んでいるが，作業者と協働可能なロボットの開発は進んでいない． 
我々は，これまでの研究において，Pan-Tilt-Zoom制御可能なカメラ，ステレオカメラといったカメラのみを用いて圃場内の作業者の行動や物体の動きを追跡・認識するシステムを開発してきた．しかしながら，カメラから距離が離れた場所で作業者や物体を検出することが困難であった．本研究では，広範囲の計測を可能とするシステムを開発するために，3D LiDARとステレオカメラを用いた作業空間内物体追跡システムを開発した．システムの開発では，3D LiDARとステレオカメラから得られる3次元点群情報を統合するためのキャリブレーション1)を行い，人物や物体を認識するために深層学習モデルの一つであるYOLOv3を用いた認識システム2)を開発し，さらに，YOLOv3を用いて検出した複数の人物や物体の空間内の移動を同時に追跡できるシステムを開発した．
本報では，開発したシステムの概要と検証実験の結果を示す．<br>
![画像1](https://user-images.githubusercontent.com/82224433/134284799-00a951a9-9a66-4ceb-b006-9fb15e2ffeac.png)



#### 7. 卒論発表会(2/17)
- [Slide](https://github.com/yuki-research/My_research/files/6286932/P_Masuda.pdf)
- [Report](https://github.com/yuki-research/My_research/files/6286943/3_.pdf)

##### はじめに
近年，スマート農業の旗印の下で，大規模稲作向けのロボットや収穫ロボットの開発が進んでいるが，作業者と協働可能なロボットの開発は進んでいない．ロボットを安全に稼働させるために，作業者や障害物を検出する技術は開発されているが，作業者の行動を理解しながら作業の支援内容を意思決定するシステムは開発されていない．
我々は，これまでの研究において，単眼カメラやPan-Tilt-Zoom制御可能なカメラ，ステレオカメラといったカメラのみを用いて圃場内の作業者の行動や物体の動きを追跡・認識するシステムを開発してきた．しかしながら，カメラの計測可能な空間は大きくないことから，カメラから距離が離れた場所で作業者や物体を検出することが困難であった．本研究では，広範囲の計測を可能とするシステムを開発するために，3D LiDARとステレオカメラを用いた作業空間内物体追跡システムを開発した．システムの開発では，3D LiDARとステレオカメラから得られる3次元点群情報を統合するためのキャリブレーション1)を行い，人物や物体を認識するために深層学習モデルの一つであるYOLOv3を用いた認識システム2)を開発し，さらに，YOLOv3を用いて検出した複数の人物や物体の空間内の移動を同時に追跡できるシステムを開発した．
本報では，第2章で開発したシステムの概要を説明し，第3章で検証実験の概要を示す．第4章で実験結果を示しながらシステムの有効性を示し，第5章でまとめを行う．






#### 6. IoUを用いた物体追跡(1/29)
- [Slide](https://github.com/yuki-research/My_research/files/6286955/20210129_P_Masuda.pdf)
- [Report](https://github.com/yuki-research/My_research/files/6286962/20210129_W_Masuda.pdf)


##### 要旨
今回の実験では，median，IoUの2種類の手法を使い，Bounding Box間のクラスタリングを行った．medianでは，Bounding Box内に存在するLiDARの3次元点群から算出した距離を使い，IoUでは，各物体のBounding Boxの座標値を使うことで，クラスタリングを行った．medianを使った場合，Bounding Boxによって背景に写像された点群が代表距離になることがあるため，良好にクラスタリングを行うことが出来なかった．一方，IoUを用いた場合，今回の実験環境においては良好なクラスタリングを得ることが出来た．今後は，異なる物体のBounding Boxが重なる場合の処理を追加する．

##### IoUによる追跡
IoU（Intersection over Union）とは，2つの領域の重なり度合いを表す指標で，領域の共通部分（Intersection）を領域の和集合（Union）で割った値である．IoUは，最大値（2つの領域が一致している）が1，最小値（2つの領域が全く重なっていない）が0である．今回の実験では，各物体のBounding Box間のIoUを算出し，最大かつ指定した閾値以上の場合に共通のクラスに分類する．<br>
![図1](https://user-images.githubusercontent.com/71996012/106340186-4bdc7b80-62dc-11eb-9482-288d1d0f50fb.png)

##### 実験結果
- Personの追跡結果<br>
![image](https://user-images.githubusercontent.com/71996012/106340265-89410900-62dc-11eb-93c8-c20806a71994.png)
- Containerの追跡結果<br>
![image](https://user-images.githubusercontent.com/71996012/106340295-a1188d00-62dc-11eb-9579-e0f5c1f59061.png)



#### 5.Bounding box内に存在するLiDAR点群を用いた代表距離算出（1/8）
- [Slide](https://github.com/yuki-research/My_research/files/6286968/20210108_P_Masuda.pdf)
- [Report](https://github.com/yuki-research/My_research/files/6286971/20210108_W_Masuda.pdf)

##### 要旨
今回の実験では，YOLOv3によって出力されたBounding boxの枠内に存在するLiDAR点群を用いて，センサから物体までの距離を算出した．Bounding boxの枠内に存在するLiDAR点群をhistogram化し，最も投票数が多い距離を代表点とすることで，代表距離を算出することは出来たが，Bounding box内に占める物体の割合が少ない場合に，背景に写像された点群が代表距離になることが分かった．今後は，背景を含めずにデータセットを囲む必要がある．

##### Bounding boxの枠内に存在するLiDAR点群から距離を算出
Bounding boxの枠内に存在するLiDAR点群のみを抽出し，抽出したLiDAR点群の2次元座標に対応する3次元座標から距離を求める．<br>![image](https://user-images.githubusercontent.com/71996012/104003151-6479da00-51e5-11eb-8653-b20d99922c44.png)

##### 抽出したLiDAR点群のhistogram化と代表点の算出
求めた距離を小数点第一位で四捨五入し，整数化する．そして，整数化した距離(1m刻み)ごとに投票を行うことで，histogram化を行い，最も投票数が多い距離を代表距離とする．<br>![image](https://user-images.githubusercontent.com/71996012/104003307-93904b80-51e5-11eb-8d83-a4537feaca96.png)

##### 実験結果
- YOLOv3で物体を学習させるために撮影した場所と同じ場所で検証を行った結果<br>![image](https://user-images.githubusercontent.com/71996012/104003416-bc184580-51e5-11eb-9336-2b5994c939f5.png)<br>![image](https://user-images.githubusercontent.com/71996012/104003457-c9353480-51e5-11eb-808e-ed017ad3bb74.png)



- YOLOv3で物体を学習させるために撮影した場所と異なる場所で検証を行った結果<br>![image](https://user-images.githubusercontent.com/71996012/104003505-d4886000-51e5-11eb-832f-1a3cb6e22240.png)<br>![image](https://user-images.githubusercontent.com/71996012/104003531-dc480480-51e5-11eb-80d2-e483c3e90bbd.png)





#### 4.Bounding box内に存在するLiDARの点群抽出（12/4）
- [Slide](https://github.com/yuki-research/My_research/files/6286973/20201204_P_Masuda.pdf)
- [Report](https://github.com/yuki-research/My_research/files/6286979/20201204_W_Masuda.pdf)

##### 要旨
今回の実験では，LiDARの設置高さによって，点群結果の違いを考察するとともに，Stereo Cameraで取得した画像からYOLOv3を用いて，物体の検出を行い，YOLOv3によって囲まれたBounding box内に存在するLiDARの点群抽出を行った．LiDARの設置高さを低くし，上方に傾けることで，良好な結果が得られるのではないかという結論に至った．また，Bounding box内のLiDARの点群抽出にかんしては，Bounding boxの座標値とLiDARの点群座標を用いることで，良好な抽出結果を得ることが出来た．今後は，抽出したLiDARの点群結果を用いて，Bounding boxで囲まれた物体までの距離を求める．

##### Bounding box内に存在するLiDARの点群を抽出する方法
YOLOv3で物体を検出した際に出力されるBounding boxの座標値とLiDARの点群座標を用いる．取得したLiDARの全座標値からBounding boxの座標値内にあるLiDARの点群座標のみを取り出す処理を行うことで，Bounding box内に存在するLiDARの点群を抽出する．<br>
![image](https://user-images.githubusercontent.com/71996012/101168399-bc6d7000-367e-11eb-83d7-84e58cd43615.png)


##### 実験結果
- 地面からLiDARまでの高さが1.0mの場合のLiDARの点群を画像に写像した結果 + LiDARのラインの推定結果<br>
![image](https://user-images.githubusercontent.com/71996012/101168721-2dad2300-367f-11eb-8135-71c29514e632.png)
![image](https://user-images.githubusercontent.com/71996012/101168992-a3b18a00-367f-11eb-8e9a-5468c1cd6e5b.png)


- 地面からLiDARまでの高さが1.6mの場合のLiDARの点群を画像に写像した結果 + LiDARのラインの推定結果<br>
![image](https://user-images.githubusercontent.com/71996012/101168780-46b5d400-367f-11eb-9a56-25e8766ef147.png)
![image](https://user-images.githubusercontent.com/71996012/101169019-af9d4c00-367f-11eb-9d2f-777c7ba7aff5.png)


- 地面からLiDARまでの高さが1.0mの場合のYOLOの結果<br>
![image](https://user-images.githubusercontent.com/71996012/101169292-0f93f280-3680-11eb-9097-207189a30ea6.png)

- 地面からLiDARまでの高さが1.6mの場合のYOLOの結果<br>
![image](https://user-images.githubusercontent.com/71996012/101169357-220e2c00-3680-11eb-9e53-d6d0e327746d.png)


- 地面からLiDARまでの高さが1.0mの場合のBounding box内のLiDARの点群抽出結果<br>
![image](https://user-images.githubusercontent.com/71996012/101169454-4833cc00-3680-11eb-991f-3f21165da7b5.png)

- 地面からLiDARまでの高さが1.6mの場合のBounding box内のLiDARの点群抽出結果<br>
![image](https://user-images.githubusercontent.com/71996012/101169484-584bab80-3680-11eb-9eba-add895a278e7.png)






#### 3.チェスボードを用いたLiDARとStereo Cameraのキャリブレーション（11/13）
- [Slide](https://github.com/yuki-research/My_research/files/6286986/20201113_P_Masuda.pdf)
- [Report](https://github.com/yuki-research/My_research/files/6286988/20201113_W_Masuda.pdf)

##### 要旨
チェスボードを活用することで，LiDARとStereo Cameraのキャリブレーションを行い，キャリブレーション結果の有効性について考察を行った．それぞれのセンサからチェスボードの交点を抽出し，交点の座標を算出することで良好なキャリブレーシ結果を得ることできた．今後は，物体の検出と識別を同時に行えるYOLOと組み合わせることで，カメラから遠い場所に存在する物体の測距と認識を行うシステムを開発したい．


##### LiDARとカメラのキャリブレーション
本研究では，Stereo CameraとLiDAR各々でチェスボードの交点を検出し，Stereo Cameraから得られる交点の座標とLiDARから得られる交点の座標を算出する．そして，各センサから取得されるチェスボードの交点の座標から回転行列 r と並行行列 t を求めことで，センサ間の統合を実現する．

##### LiDARにおける交点検出
LiDARとは，レーザーを物体に照射し，反射した結果から距離を計算する技術である．LiDARは，物体の色によって反射強度が異なるという特性があり，白色の物体に対する反射強度は高く，黒色の物体に対する反射強度は低くなる．今回，取得した反射強度の値をヒストグラムに表示することで，チェスボード内の白色と黒色の領域を判断し，交点の座標を抽出する．

##### カメラにおける交点検出
本研究では，チェスボードをStereo Cameraで撮影し，取得した画像データからチェスボードの交点の座標を算出する．その際，画像処理の機能を持つライブラリであるOpenCVのfindChessboardCorners()関数を用いる．画像データから交点の座標を算出した後，三角測量により3次元座標を取得する．<br>
![image](https://user-images.githubusercontent.com/71996012/99097418-f4067080-261a-11eb-8f0e-93e33096b5d7.png)

##### 実験結果
- OpenCVのfindChessboardCorners()関数を使って，チェスボードの交点を検出した結果<br>
![image](https://user-images.githubusercontent.com/71996012/99097832-83138880-261b-11eb-8b39-863cd2c2357e.png)

- LiDARで検出した反射強度をヒストグラムに表示した結果<br>
![image](https://user-images.githubusercontent.com/71996012/99097965-ae967300-261b-11eb-8896-746f178cc3d9.png)

- 反射強度の結果から，チェスボードの白黒領域と交点を検出した結果<br>
![image](https://user-images.githubusercontent.com/71996012/99098079-d554a980-261b-11eb-8948-f6c639dee335.png)

- 画像内のチェスボード上にLiDARの点群を写像した結果<br>
![image](https://user-images.githubusercontent.com/71996012/99098146-ef8e8780-261b-11eb-9606-5f71fa436dbc.png)



#### 2.ORB-SLAMによる自己位置推定と環境地図作成（10/23）
- [Slide](https://github.com/yuki-research/My_research/files/6286991/20201023_P_Masuda.pdf)
- [Report](https://github.com/yuki-research/My_research/files/6286995/20201023_W_Masuda.pdf)

##### 要旨
現在，農業従事者の後継者不足が問題となっている．また，農作業現場において，作業者と協調しながら作業を行うロボットの開発が求められている．そこで，私はロボット自身が自分の位置を推定し，地図を作成する技術を活用することで，自律走行を可能とする環境認識システムの開発を行う．本研究では，距離情報を計算することが出来るStereo cameraを用いて，倉庫内（金工室内）を走行し，実際の経路や距離とORB-SLAMによって推定した結果の比較を行った．距離にかんしては誤差が少なかったが，経路にかんしては比較的誤差が大きいという結果が得られた．今後は，誤差が生じた箇所に特徴となる物体を配置し，実際の経路との誤差を少なくしていきたい．

##### SLAMとは？
SLAM（Simultaneous Localization and Mapping）とは，自己位置推定と環境地図作成を同時に行う技術のことを言う．図3よりSLAMを用いることで，自分の居場所や周辺の環境を把握することができるため，自動運転やドローン，ロボットなどで活用されている．SLAMには，LiDAR（レーザースキャナ）から取得したデータによって実現するLiDAR SLAM，センタから取得した深度画像（距離情報）によって実現するDepth SLAM，カメラからの画像によって実現するVisual SLAMが存在する．今回は，ステレオカメラを用いて走行したため，Visual SLAMの一種であるORB-SLAMを用いて研究を行う1)．

##### ORB-SLAMとは？
ORB-SLAMとは，カメラで取得した画像からコーナーのみを特徴点として検出する手法で，重度の動きの乱れに対して強く，自己位置推定や周辺地図の作成，Loop Closingなどのタスクで優れた性能を持つ2)．ちなみに，Loop Closingとは過去に見た風景と認識された場合にこれまでに蓄積された地図の誤差を除去するように地図を更新する機能である．
- 実験の走行経路<br>
![image](https://user-images.githubusercontent.com/71996012/97063471-e3924580-15da-11eb-8065-b46cd8e850e6.png)
![image](https://user-images.githubusercontent.com/71996012/97063479-edb44400-15da-11eb-9534-0c77eb2baae2.png)<br>

- 特徴点の抽出と推定した経路（ORB-SLAM）<br>
![image](https://user-images.githubusercontent.com/71996012/97063314-f5bfb400-15d9-11eb-9bc8-402848950068.png)<br>
- ORB-SLAMによって推定した経路をgnuplotで表示<br>
![image](https://user-images.githubusercontent.com/71996012/97063376-5d75ff00-15da-11eb-911d-c1075903fc45.png)


#### 1.夏季休暇中の取り組み（10/2）
- [Slide](https://github.com/yuki-research/My_research/files/6287000/20201002_P_Masuda.pdf)
- [Report](https://github.com/yuki-research/My_research/files/6287002/1.pdf)

##### 要旨
夏休みは，『大学院試験の勉強』・『情報セキュリティマネジメント試験の勉強』・『卒論の準備』を行ってきました．
##### 参考記事
- [ロバスト推定法とは？](https://www.eyedeal.co.jp/robustestimation.html)
- [KL展開とは？](https://www.iwanttobeacat.com/entry/2018/06/16/150224)
- [ステレオカメラ技術の原理・技術](https://www.fsi-embedded.jp/product/fsi/stereo-vision/technique/)


----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

### ２０２０前期（4年）
#### 5.膨張・収縮処理による影響とHough変換を用いた直線検出（最終発表）
- [Slide](https://github.com/yuki-research/My_research/files/6287010/0710_P_Masuda.pdf)
- [Report](https://github.com/yuki-research/My_research/files/6287013/0710_W_Masuda.pdf)

##### 要旨
画像内の道路沿いを抽出し，尚且つ抽出した道路沿いから直線を検出する際に，輪郭抽出技術やHough変換を使うことの有効性について考察を行った．本稿では，前回の研究で行った膨張・収縮処理による影響について述べるとともに，抽出した道路沿いのエッジとHough変換によって検出した直線を活用し，試作品を考えた．
##### Hough変換とは？
ハフ(Hough)変換は、画像の中から直線や円などの図形を検出したい際によく用いられる手法の一つです。検出精度は高いですが、その分演算負荷が高く、またメモリも多く消費するなどの欠点があります。

![図1](https://user-images.githubusercontent.com/71996012/94911723-875b6a80-04e1-11eb-9d8c-db11e703ac56.png)

#### 4.median filterと輪郭追跡処理によるノイズ除去（5回目の発表）
- [Slide](https://github.com/yuki-research/My_research/files/6287017/0626_P_Masuda.pdf)
- [Report](https://github.com/yuki-research/My_research/files/6287018/0626_W_Masuda.pdf)

##### 要旨
画像内の道路沿いを抽出する際に，輪郭抽出技術を使うことの有効性について考察を行った．本稿では，median filterのサイズを大きくし，さらに，輪郭追跡処理を用いることで不要な輪郭の除去を試みた．今回紹介する手法を用いることで，不要な輪郭を除去し，道路沿いのみの輪郭を抽出することが可能となった．

##### 輪郭追跡処理とは？
輪郭追跡処理とは，二値化された画像において，各連結部分の境界部分をもとめる手法である．輪郭追跡処理を行うことで，周辺の長さや，連結領域の幅・高さを求めることが可能である．

![0626_P_Masuda](https://user-images.githubusercontent.com/71996012/94912771-22087900-04e3-11eb-80c7-845719cd6fe7.png)
![0626_P_Masuda2](https://user-images.githubusercontent.com/71996012/94912993-6e53b900-04e3-11eb-9fec-166c26f23db4.png)

##### メディアンフィルタとは？
median filterとは，画像のノイズを除去する手法の1つで，注目画素を，周りの画素の濃度の中央値に変換することである．

![メディアンフィルタ_サイズごとの結果の確認用](https://user-images.githubusercontent.com/71996012/94912858-3c425700-04e3-11eb-807e-bc3a8ff973e4.png)


#### 3.画像内の道路のHSV値抽出とノイズ除去（4回目の発表）
- [Slide](https://github.com/yuki-research/My_research/files/6287023/0612_P_masuda.pdf)
- [Report](https://github.com/yuki-research/My_research/files/6287024/0612_W_Masuda.pdf)

##### 要旨
画像内の道路沿いを検出する際に，輪郭検出技術を使うことの有効性について考察を行った．本稿では，指定した範囲のHSV値の抽出方法を改善し，さらに，median filterやオープニング処理を用いて，不要な輪郭の除去を試みた．今回紹介する手法を用いることで，前回の結果からさらに不要な輪郭を除去することが可能となった．

##### HSV値抽出とは？
まず元画像と元画像を用いて作成したモノクロ画像（paint.netを使用）を用意し，それらの画像を照らし合わせる．次に，モノクロ画像の白の部分に対応する元画像の範囲のHSV値をヒストグラムに表す．そして，ヒストグラムから最小閾値と最大閾値を決定し，最大閾値よりも大きい場合と小さい場合は，それらの画素の値を0（黒）に変換し，閾値以内の場合は，255（白）に変換する．

![0605_P_masuda2](https://user-images.githubusercontent.com/71996012/94916190-2f286680-04e9-11eb-8e5b-928e97f82816.png)

##### ノイズ除去で使った手法→オープニング処理とは？
オープニング処理とは，収縮処理と膨張処理を組み合わせた動作である．二値化画像を収縮した回数だけ膨張することで，大きさを変えずに，粒子を引き離すことが可能となる．

![0612_P_masuda](https://user-images.githubusercontent.com/71996012/94915934-b1645b00-04e8-11eb-8b20-b7eca686f11d.png)

#### 2.画像内の道路沿いを抽出（3回目の発表）
- [Slide](https://github.com/yuki-research/My_research/files/6287029/0522_P_Masuda.pdf)
- [Report](https://github.com/yuki-research/My_research/files/6287031/0522_W_Masuda.pdf)

##### 要旨
画像内の道路沿いを検出する際に，輪郭検出技術を使うことの有効性について考察を行った．本稿では，農業現場の画像の上部にマスク処理を施し，RGB値やHSV値による閾値処理を行った後に輪郭を検出した．研究結果から，不要な輪郭を除去するためにマスク処理は有効であり，またRGB値やHSV値による色抽出を行うことで，鮮明に輪郭を検出することができるということが分かった．

##### RGB値やHSV値による閾値処理
画像内の道路のみを抽出するために，RGB値による閾値処理を試みる．まず，最小閾値と最大閾値を指定する．そして，カラー画像のRGB値が最大閾値よりも大きい場合と小さい場合は，0（黒）に変換し，閾値以内の場合は，255（白）に変換する．同様の方法で，HSV値による閾値処理も試みる．HSV値を使うことで，色相・色彩・明度を表現することが出来る．

![0522_P_Masuda](https://user-images.githubusercontent.com/71996012/94916603-f2a93a80-04e9-11eb-8b16-1e5f612dee00.png)

##### Gaussian filterによる平滑化
処理対象となる注目画素の重みを最も大きくし，外側の画素の重みを小さくする特性がある．この特性により，注目画素付近の情報を残し平滑化を行うことが可能となる．

![0501_P_Masuda](https://user-images.githubusercontent.com/71996012/94916675-1a000780-04ea-11eb-876c-12322edbcf7c.png)

##### Sobel filterによる輪郭抽出
Sobel filterを用いて，画像内の輪郭を抽出する．この手法を使うことで，輝度差の少ない輪郭も強調されるため，今回用いる画像の輪郭抽出に適している．この手法は，縦方向と横方向の2つのフィルターを用いて畳み込み演算を行い，得られた2つの値をそれぞれ2乗し，平方根で表した結果が，処理後の画像の画素値となる．

![0501_P_Masuda2](https://user-images.githubusercontent.com/71996012/94916846-651a1a80-04ea-11eb-9486-196f0646f7e9.png)

#### 1.画像処理による輪郭検出（2回目の発表）
- [Slide](https://github.com/yuki-research/My_research/files/6287034/0501_P_Masuda.pdf)
- [Report](https://github.com/yuki-research/My_research/files/6287036/0501_W_Masuda.pdf)

##### 要旨
画像内の道路を検出する際に，輪郭検出技術を使うことの有効性について考察を行った．本稿では，農業現場の画像をグレイスケールに変換し，平滑化処理を行った後に輪郭を検出した．さらに，二値化処理で輪郭をより鮮明にし，細線化処理によって輪郭を一本の線で表現した．処理結果から道路沿いの輪郭検出の精度が良好ではないので，今後改善が求められる．

##### 大津の二値化とは？
まず画像の画素数をヒストグラムに表す．そして，2つのクラスに分けて，クラス同士の離れ具合（クラス間分散）が最大になる時の閾値で二値化する手法を大津の二値化という．

![0501_P_Masuda3](https://user-images.githubusercontent.com/71996012/94917140-f4273280-04ea-11eb-8d73-dca0540015b9.png)
