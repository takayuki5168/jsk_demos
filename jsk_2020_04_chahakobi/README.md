# 茶運びロボット

PR2がお茶を作って持ってきてくれるデモを作る。
お湯をわかせて注げれば、お茶だけでなく、コーヒー、カップラーメン、スープ等にも対応できる。

## 作るデモ
### IRT viewer  
IRT viewer上のデモ。
```
roscd jsk_2020_04_chahakobi/euslisp/demos
roseus irt-demo.l
```

###  kinematics simulater  
運動学シミュレータを使って実機と同じeusのインターフェースで作ったデモ。
```
roscd jsk_2020_04_chahakobi/euslisp/demos
roseus kinematics-demo.l
```

### Gazebo  
物理シミュレーションが大事になりそうな部分について、Gazeboを使ってシミュレーションを行う。

キッチンの環境を立ち上げる。
```
roslaunch jsk_2020_04_chahakobi gazebo_test.launch
```

テーブルの上のやかんをICPで認識して掴む
```
roslaunch jsk_2020_04_chahakobi grasp_kettle.launch

roslaunch jsk_2020_04_chahakobi icp_kettle.launch

roscd jsk_2020_04_chahakobi/euslisp/
roseus grasp-kettle.l
```

WIP これから実現するgazeboシミュレーション。
- 73b2の環境を使って行うようにする。
- やかんをglobalなRegistrationからつかむ。
- (干渉などを気にしながら)やかんに水をいれる。
- やかんをもってコップにお湯を注ぐ。


## デモの流れ

- コンロまで行く (go-to-cook)
- やかんの蓋を取る (open-the-lid)
- やかんを持つ (pick-up-kettle)
- シンクまで移動する (go-to-sink)
- やかんに水を入れる (pour-water)
- コンロの前に行く (go-to-cook2)
- やかんをコンロに置く (put-kettle)
- 蓋を閉じる (close-the-lid)
- コンロをつける (put-on-stove)
- お茶を用意する (prepare-tea)
- お湯が湧くのを待つ (wait-boil)
- コンロを消す (turn-off-stove)
- やかんを持つ (pick-up-kettle2)
- お湯を注ぐ (pour-hot-water)
- やかんを置く (put-kettle2)
- コップを持つ (pick-up-cup)
- 机まで持っていく (go-to-desk)

一つの動作を一つの関数にしてあり、一つ実行すると次に実行する関数がlogにでるようにしてある。  

また、
```lisp
(exec-all)
```
ですべての動作を実行。
```lisp
(now-devel)
```
で今作っている部分の手前までを実行できる。  

## 考えられる改善
最初にお茶やコーヒー、カップラーメンなどを用意してから始めるべき？  
eusのモデルを修正してやかんの蓋を取れるようにする？  
お茶のパックのモデルを作成する？  
お茶やコーヒーの場所をはじめから知っておくべき？そこに無かったら？もし在庫が切れていたら？  
最初に、やかんの向き、お湯が入っているのかどうか、やかんが熱く無いか等を確認する？  
