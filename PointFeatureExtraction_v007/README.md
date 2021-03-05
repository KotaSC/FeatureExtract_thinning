# PointFeatureExtraction_v007

## 概要
3次元点群データから固有値を算出し，得られた固有値から特徴量を計算する．

## 使い方
```
USAGE   : $ ./pfe [input_point_cloud_data] [output_point_cloud_data]
EXAMPLE : $ ./pfe [input_point_cloud.ply] [output_point_cloud.xyz]
```

## 使用例

```
$ ./pfe ../PLY_DATA/box/box.spbr ../XYZ_DATA/box/box.xyz
~~~~~ #/SPBR_ASCII_Data
SPBR (Ascii) file reading.....
** SPBR ascii data: ../PLY_DATA/box/box.spbr
** Repeat level is set to "1".
** #/EndHeader command is found.
** Reading the header part is completed:
**   Number of points     : 10000000
**   Use of normal vectors: Yes
**   Repeat level         : 1
** Real number of points: 10000000
** PointObject is ready.
** Point-set range:
  #/BoundingBox  0 0 0  1 1 1
PLY Mim, Max Coords:
Min : 0 0 0
Max : 1 1 1

Feature calculation type
Point PCA: 0, Minimum entropy PCA: 1
Select an ID >> 0
Feature calculation type ==> 0

Feature value type
Change of curvature: 0, Aplanarity: 1, Linearity: 2, Eigentropy: 3
Select an ID >> 0
Feature value type ==> 0

Highlighting precision
Input 1/local-area_radius (recommend range [100-600]) >> 500
Local-area radius = 0.0034641

Creating Octree... (Number of Vertex : 10000000
0 0 0
1 1 1
Start OCtree Search.....
1000000, 63: 0
2000000, 76: 0
3000000, 66: 0
4000000, 64: 0
5000000, 80: 0
6000000, 74: 0
7000000, 77: 0
8000000, 68: 0
9000000, 45: 0
10000000, 67: 0
Maximun of Sigma : 0.151453
```
