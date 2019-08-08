# FeatureExtract_thinning
特徴強調可視化プログラム

## 使い方

### ・PointFeatureExtraction_v007
入力された点群の座標値に対して主成分分析を行い，特徴量を算出するプログラムです．  
読み込み可能なデータは以下の形式になります．
- ply
- xyz
- spbr

プログラムの使い方
1. ディレクトリに移動して以下のコマンドを入力してください．コマンドが実行されると，```pfe```という名前の実行ファイルが作成されます．
```
$ make
```
2. 入力ファイルと出力ファイルを記述してください．3つめの```[output_file_name]```はオプションです，書かなかった場合には```out.xyz```というファイル名で結果が出力されます．
```
$ pfe [input_file_name] [output_file_name]
```

### ・alphaControl4PLT_withFeature_thinning
特徴量に応じて不透明度を制御するプログラムです．

```
make
```
