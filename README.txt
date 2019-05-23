特徴抽出と可視化ソフトの使い方
【注意】
Makefile内のPCLの環境は各自のPCの環境に書き換えて下さい．
書き換えるのは，Makefileの20行目付近の                
INCLUDE_PATH :=
LIBRARY_PATH :=
の２つです．

１．PointFeatureExtraction_v006
　使い方の詳細は，DOC内の「HowtoUsePointExtraction.doc」を見て下さい．
　このプログラムを実行すると，点群＋特徴量のxyz形式でファイルが出力されます．

　参考資料としてはDOC内の以下のファイルを見て下さい．
　　CAに関しては田中先生が作成された資料：
　　　　pca_010a.pptx
　　共分散行列，LAPACKに関しては卒研ゼミ資料：
　　　　cgtanaka_numer_2016_001a.pdf
　　点群特徴抽出に関しては岩切先生の論文：
　　　　110009866453.pdf　


２．DrawFeatureControlBW_003
　１で出力された特徴量について，閾値（どの値以上を特徴量として抽出するか）を
　スライダーを使って視覚的に確認するプログラムです．
　黒く成った部分を特徴量と見なします．
　いくつ以上を特徴領域としたかは，コンソール上に出力されています．
　KVSがあれば他には特に必要なライブラリはありません．
　$ make 
　でコンパイルしすると，drawFeatureという実行ファイルが作成されます．
　$ drawFeature out.xyz
　で（2つめの引数が可視化したい特徴量付き点群データです）実行できます
　　
３．alphaControl4PLY_withFeature_v002
　点群全体と特徴領域を半透明融合可視化するソフトです．
　PCLとKVSを使っています．
　$ make
　でコンパイルすると，alphaControl4ply という実行ファイルが作成されます．
　$ alphaControl4ply 
　 で実行すると，オプション一覧が表示されます．
　 基本的な使い方は
　$ alphaControl4ply 点群データ -a 全体の不透明度 -fa 特徴領域の不透明度 -ft 特徴領域の値
　です．
　ここで，特徴領域の値は，２番のプログラムで決めた値を入れます．
　　Ex) 
　　　$ alphaControl4ply bunny.xyz -a 0.1 -fa 0.8 -ft 0.02
　　　　　のように実行します
　　

【サンプルデータ】
Dataフォルダ内にいくつかサンプルをいれました．
（とりあえずbunny あたりがいいかなと思います）
bunny.xyz は点群と法線のみです


　