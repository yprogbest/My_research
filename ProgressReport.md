# 【タスク管理】
# 4月12日(月)　更新

## 現在のタスク
- YOLOv3(英語：You only look once)による物体認識
  - 〇〇
  

- LIDAR(英語：Laser Imaging Detection and Ranging)による光検出と測距
  - LiDARとステレオカメラのパラメータの算出後に統合する方法について理解出来ていなかったので，調べる（2020/11/13現在）
  - 遠方にある物体上にLiDARの点群が写像されない時の対処法を考える．
  - gnuplotで動的なグラフを作成する
  

- OpenCV
  - 〇〇
  
  
- キャリブレーション
  - コードを改めて見直す（※理解できていない箇所を潰していく）【途中】
  
 

- その他
  - cMakeの設定（何回行っても上手くいかない（opencv_buildフォルダー内に『install』というフォルダーが追加されるはずだが，追加されない．）【中断】
   
   
   
### 今週のタスク

    - Bounding box内のLiDAR点群の中央値を求める【完了】
    - 移動軌跡をgnuplotに表示【完了】
    - 移動軌跡のうち，背景を取り除く処理を考える．
    

#### 本日のタスク

    - 移動軌跡のうち，背景を取り除く処理を考える．
    
 
 
 
   
## 活動報告
### 1/12(火) Bounding box内のLiDAR点群の中央値を算出した
- 前回の発表で，Bounding box内の代表距離を算出する際に，ヒストグラムの最大値を代表距離とした．しかし，この方法では，Bounding box内に占める背景の割合が多い場合に，背景上の点群が代表距離になるという問題が発生した．そこで，今回は，Bounding box内のLiDAR点群の距離を降順に並び替え，その中央の値を代表距離とすることで，背景が代表距離になってしまうという問題を解決出来るのではないかと考えた．コードは書き終えたが，誤作動が起こっているため，後日修正を行う．<br><strong>誤作動の様子</strong><br>![image](https://user-images.githubusercontent.com/71996012/104320727-0d963c80-5526-11eb-849d-6b952f4d7b0a.png)<br>【追記】解決した（パスの指定先が異なっていたことが原因だった．）


## 1/15(金) コードを改編すると，強制終了してしまいます．
- 現在，Bounding box内のLiDARの点群の中央値を算出するためのコードを書いていて，実行すると，143番目で『続行するには何かキーを押してください』と表示され，強制終了してしまいます．
- 上の問題が解決したら，<strong>リングバッファ</strong>を使って，物体の移動量を算出する．<br>![image](https://user-images.githubusercontent.com/71996012/104681148-ff6a3b00-5734-11eb-920c-590fd0880106.png)<br>【追記】解決した（Bounding box内にLiDAR点群が存在しない時の処理を加えることで，解決した．）


### 現在の進捗状況
- Bounding box内のLiDAR点群の中央値を算出し，142番目までの結果をgnuplotに表示することが出来た．<br><strong>画像</strong><br>![image0](https://user-images.githubusercontent.com/71996012/104681351-7b648300-5735-11eb-93d3-856c3d4ee6ee.png)<br><strong>gnuplotの結果</strong><br>![image0](https://user-images.githubusercontent.com/71996012/104681419-9d5e0580-5735-11eb-979f-174ed1b00968.png)



## 今後の予定
### 今週のタスク

    - Bounding box内のLiDAR点群の中央値を求める【完了】
    - 移動軌跡をgnuplotに表示【完了】
    - 移動軌跡のうち，背景を取り除く処理を考える．
    
    
#### 明日のタスク
    
    - 移動軌跡のうち，背景を取り除く処理を考える．
    


【4/12(月) 23:00 upload】
