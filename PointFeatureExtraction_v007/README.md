# PointFeatureExtraction_v007

## 概要
3次元点群データから固有値を算出し，得られた固有値から特徴量を計算する．

## 使い方
```
USAGE   : $ ./pfe [input_point_cloud_data] [output_point_cloud_data]
EXAMPLE : $ ./pfe [input_point_cloud.ply] [output_point_cloud.xyz]
```

## 使用例1

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
## 使用例2
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
Select an ID >> 1
Feature calculation type ==> 1

Feature value type
Change of curvature: 0, Aplanarity: 1, Linearity: 2, Eigentropy: 3
Select an ID >> 0
Feature value type ==> 0

Highlighting precision
Input minimum 1/local-area_radius (recommend range [100-600]) >> 300
Input maximum 1/local-area_radius (recommend range [100-600]) >> 500
Input Number of calculations >> 5
Minimum local-area radius = 0.0034641
Maximum local-area radius = 0.0057735

Start calculation 1
Local-area radius = 0.0034641
Creating Octree... (Number of Vertex : 10000000
0 0 0
1 1 1
Start OCtree Search.....
1000000, 63 EigenValues: ( 2.99063e-06, 2.78481e-06, 0 )
2000000, 76 EigenValues: ( 2.99063e-06, 2.78481e-06, 0 )
3000000, 66 EigenValues: ( 2.99063e-06, 2.78481e-06, 0 )
4000000, 64 EigenValues: ( 2.99063e-06, 2.78481e-06, 0 )
5000000, 80 EigenValues: ( 2.99063e-06, 2.78481e-06, 0 )
6000000, 74 EigenValues: ( 2.99063e-06, 2.78481e-06, 0 )
7000000, 77 EigenValues: ( 2.99063e-06, 2.78481e-06, 0 )
8000000, 68 EigenValues: ( 2.99063e-06, 2.78481e-06, 0 )
9000000, 45 EigenValues: ( 2.99063e-06, 2.78481e-06, 0 )
10000000, 67 EigenValues: ( 2.99063e-06, 2.78481e-06, 0 )
1000000, Feature Value: 0, Eigentropy: 0
2000000, Feature Value: 0, Eigentropy: 0
3000000, Feature Value: 0, Eigentropy: 0
4000000, Feature Value: 0, Eigentropy: 0
5000000, Feature Value: 0, Eigentropy: 0
6000000, Feature Value: 0, Eigentropy: 0
7000000, Feature Value: 0, Eigentropy: 0
8000000, Feature Value: 0, Eigentropy: 0
9000000, Feature Value: 0, Eigentropy: 0
10000000, Feature Value: 0, Eigentropy: 0
Start calculation 2
Local-area radius = 0.00404145
Creating Octree... (Number of Vertex : 10000000
0 0 0
1 1 1
Start OCtree Search.....
1000000, 85 EigenValues: ( 4.21835e-06, 3.48538e-06, 0 )
2000000, 99 EigenValues: ( 4.21835e-06, 3.48538e-06, 0 )
3000000, 86 EigenValues: ( 4.21835e-06, 3.48538e-06, 0 )
4000000, 81 EigenValues: ( 4.21835e-06, 3.48538e-06, 0 )
5000000, 98 EigenValues: ( 4.21835e-06, 3.48538e-06, 0 )
6000000, 101 EigenValues: ( 4.21835e-06, 3.48538e-06, 0 )
7000000, 98 EigenValues: ( 4.21835e-06, 3.48538e-06, 0 )
8000000, 86 EigenValues: ( 4.21835e-06, 3.48538e-06, 0 )
9000000, 63 EigenValues: ( 4.21835e-06, 3.48538e-06, 0 )
10000000, 90 EigenValues: ( 4.21835e-06, 3.48538e-06, 0 )
1000000, Feature Value: 0, Eigentropy: 0
2000000, Feature Value: -2.27798e-20, Eigentropy: 0
3000000, Feature Value: 0, Eigentropy: 0
4000000, Feature Value: 0, Eigentropy: 0
5000000, Feature Value: 0, Eigentropy: 0
6000000, Feature Value: 0, Eigentropy: 0
7000000, Feature Value: 0, Eigentropy: 0
8000000, Feature Value: 0, Eigentropy: 0
9000000, Feature Value: 0, Eigentropy: 0
10000000, Feature Value: 0, Eigentropy: 0
Start calculation 3
Local-area radius = 0.0046188
Creating Octree... (Number of Vertex : 10000000
0 0 0
1 1 1
Start OCtree Search.....
1000000, 112 EigenValues: ( 5.06221e-06, 4.58729e-06, 0 )
2000000, 114 EigenValues: ( 5.06221e-06, 4.58729e-06, 0 )
3000000, 107 EigenValues: ( 5.06221e-06, 4.58729e-06, 0 )
4000000, 114 EigenValues: ( 5.06221e-06, 4.58729e-06, 0 )
5000000, 118 EigenValues: ( 5.06221e-06, 4.58729e-06, 0 )
6000000, 125 EigenValues: ( 5.06221e-06, 4.58729e-06, 0 )
7000000, 124 EigenValues: ( 5.06221e-06, 4.58729e-06, 0 )
8000000, 129 EigenValues: ( 5.06221e-06, 4.58729e-06, 0 )
9000000, 88 EigenValues: ( 5.06221e-06, 4.58729e-06, 0 )
10000000, 107 EigenValues: ( 5.06221e-06, 4.58729e-06, 0 )
1000000, Feature Value: 0, Eigentropy: 0
2000000, Feature Value: 0, Eigentropy: 0
3000000, Feature Value: 0, Eigentropy: 0
4000000, Feature Value: 0, Eigentropy: 0
5000000, Feature Value: 0, Eigentropy: 0
6000000, Feature Value: 0, Eigentropy: 0
7000000, Feature Value: 0, Eigentropy: 0
8000000, Feature Value: 0, Eigentropy: 0
9000000, Feature Value: 0, Eigentropy: 0
10000000, Feature Value: 0, Eigentropy: 0
Start calculation 4
Local-area radius = 0.00519615
Creating Octree... (Number of Vertex : 10000000
0 0 0
1 1 1
Start OCtree Search.....
1000000, 143 EigenValues: ( 6.48031e-06, 6.21622e-06, 0 )
2000000, 142 EigenValues: ( 6.48031e-06, 6.21622e-06, 0 )
3000000, 132 EigenValues: ( 6.48031e-06, 6.21622e-06, 0 )
4000000, 147 EigenValues: ( 6.48031e-06, 6.21622e-06, 0 )
5000000, 144 EigenValues: ( 6.48031e-06, 6.21622e-06, 0 )
6000000, 147 EigenValues: ( 6.48031e-06, 6.21622e-06, 0 )
7000000, 156 EigenValues: ( 6.48031e-06, 6.21622e-06, 0 )
8000000, 157 EigenValues: ( 6.48031e-06, 6.21622e-06, 0 )
9000000, 124 EigenValues: ( 6.48031e-06, 6.21622e-06, 0 )
10000000, 143 EigenValues: ( 6.48031e-06, 6.21622e-06, 0 )
1000000, Feature Value: 0, Eigentropy: 0
2000000, Feature Value: -7.53156e-18, Eigentropy: 0
3000000, Feature Value: 0, Eigentropy: 0
4000000, Feature Value: 0, Eigentropy: 0
5000000, Feature Value: 0, Eigentropy: 0
6000000, Feature Value: 0, Eigentropy: 0
7000000, Feature Value: 0, Eigentropy: 0
8000000, Feature Value: 0, Eigentropy: 0
9000000, Feature Value: 0, Eigentropy: 0
10000000, Feature Value: 0, Eigentropy: 0
Start calculation 5
Local-area radius = 0.0057735
Creating Octree... (Number of Vertex : 10000000
0 0 0
1 1 1
Start OCtree Search.....
1000000, 174 EigenValues: ( 8.20748e-06, 7.48201e-06, 0 )
2000000, 175 EigenValues: ( 8.20748e-06, 7.48201e-06, 0 )
3000000, 166 EigenValues: ( 8.20748e-06, 7.48201e-06, 0 )
4000000, 183 EigenValues: ( 8.20748e-06, 7.48201e-06, 0 )
5000000, 183 EigenValues: ( 8.20748e-06, 7.48201e-06, 0 )
6000000, 187 EigenValues: ( 8.20748e-06, 7.48201e-06, 0 )
7000000, 186 EigenValues: ( 8.20748e-06, 7.48201e-06, 0 )
8000000, 189 EigenValues: ( 8.20748e-06, 7.48201e-06, 0 )
9000000, 148 EigenValues: ( 8.20748e-06, 7.48201e-06, 0 )
10000000, 177 EigenValues: ( 8.20748e-06, 7.48201e-06, 0 )
1000000, Feature Value: 0, Eigentropy: 0
2000000, Feature Value: 0, Eigentropy: 0
3000000, Feature Value: 0, Eigentropy: 0
4000000, Feature Value: 0, Eigentropy: 0
5000000, Feature Value: 0, Eigentropy: 0
6000000, Feature Value: 0, Eigentropy: 0
7000000, Feature Value: 0, Eigentropy: 0
8000000, Feature Value: 0, Eigentropy: 0
9000000, Feature Value: 0, Eigentropy: 0
10000000, Feature Value: 0, Eigentropy: 0
Maximun of Sigma : 0.141797