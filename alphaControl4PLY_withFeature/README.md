# alphaControl4PLY_withFeature

## 概要
入力点群のエッジを強調して可視化する．
## 使い方
```
USAGE   : ./alphaControl4ply [input_point_cloud_data_with_feature_value] [output_directory_name]
EXAMPLE : ./alphaControl4ply [input_point_cloud.xyz] [output_directory]
```

## 使用例1
```
$ ./alphaControl4ply ../XYZ_DATA/box/box.xyz ../SPBR_DATA/box

Input parameters
Repeat Level LR >> 100
Image resolution >> 1024
Feature value threshold f_th in range [0, 1] >> 0.03
Minimum opacity α_min in range [0, 1] >> 0.2

~~~~~ 0.801085 1 0.40658 0 0 0 255 255 255 0
XYZRGB file or Other type file reading.....
Number of points: 10000000
Input data Size: 0, Min: 0 0 0, Max: 1 1 1
Bounding Box size [ MIN: 0 0 0, MAX: 1 1 1 ]
~~~~~ 0.801085 1 0.40658 0 0 0 255 255 255 0
XYZRGB file or Other type file reading.....
Number of points: 10000000

=============================================================
Creating Particles from: ../XYZ_DATA/box/box.xyz
L: 0.00161802
PLY data dosen't have polygons
Creating Octree... (Number of Vertex : 10000000
0 0 0
1 1 1
Start OCtree Search.....
==========================================
Ana: 32121, execNum: 6407.63
Dist; 0.000909243, offset: 1.40516
anaNumfromDreal: 4560.07
Ratio_Ana: 5.01293
==========================================
Ratio for Alpha Control : 7.04397, Diff : -25713.4
Number of Original Vertices : 10000000
Number of setting Particles : 70439717( 70000000 + 439717 )
Number of Particles: 69546588
10000000	particles (14.3788) %
20000000	particles (28.7577) %
30000000	particles (43.1366) %
40000000	particles (57.5154) %
50000000	particles (71.8943) %
60000000	particles (86.2731) %
** File ../SPBR_DATA/../SPBR_DATA/box/out-box.xyz.spbr  is generated.

Feature extraction type
Normal point feature extraction: 0, Adaptive point feature extraction: 1
Select an ID >> 0
Feature extraction type ==> 0


Highlight color
ORIGINAL: 0, RED: 1, BLACK: 2, CYAN: 3
Select an ID >> 1
Highlight color ==> 1


Input opacity function parameters
Maximum opacity α_max in range [0.2, 1] >> 1
Large feature value threshold F_th in range [0.03, 1] >> 1
Point proliferation exponent d >> 2

Write a parameter list ParameterList.txt

** File ../SPBR_DATA/../SPBR_DATA/box/out-box.xyz.spbr_f.spbr  is generated.
Number of Particles (Total): 71285156

Keyboard menu:
  o-key: object control, l-key: light control
  s-key: snapshot image (BMP)
  S-key: snapshot image (PPM)
  G-key: snapshot image (PGM)
  q-key: quit
```

## 使用例2
```
kawakamikota@KotaMBP alphaControl4PLY_withFeature % ./alphaControl4ply ../XYZ_DATA/box/box.xyz ../SPBR_DATA/box

Input parameters
Repeat Level LR >> 100
Image resolution >> 1024
Feature value threshold f_th in range [0, 1] >> ^C
kawakamikota@KotaMBP alphaControl4PLY_withFeature % ./alphaControl4ply ../XYZ_DATA/mixed_edge_box/mixed_edge_box.xyz ../SPBR_DATA/mixed_edge_box/

Input parameters
Repeat Level LR >> 100
Image resolution >> 1024
Feature value threshold f_th in range [0, 1] >> 0.^C
kawakamikota@KotaMBP alphaControl4PLY_withFeature % ./alphaControl4ply ../XYZ_DATA/borobudur/relief_ship/relief_ship.xyz ../SPBR_DATA/borobudur/relief_ship/

Input parameters
Repeat Level LR >> 100
Image resolution >> 1024
Feature value threshold f_th in range [0, 1] >> 0.02
Minimum opacity α_min in range [0, 1] >> 0.2

~~~~~ -7.52888 -0.808788 -2.29837 0 0 0 134 124 106 0.00277798
XYZRGB file or Other type file reading.....
Number of points: 4635575
Input data Size: 0, Min: -7.54352 -0.852543 -2.35102, Max: -6.01018 -0.00341684 -2.23021
Bounding Box size [ MIN: -7.54352 -0.852543 -2.35102, MAX: -6.01018 -0.00341684 -2.23021 ]
~~~~~ -7.52888 -0.808788 -2.29837 0 0 0 134 124 106 0.00277798
XYZRGB file or Other type file reading.....
Number of points: 4635575

=============================================================
Creating Particles from: ../XYZ_DATA/borobudur/relief_ship/relief_ship.xyz
L: 0.00248098
PLY data dosen't have polygons
Creating Octree... (Number of Vertex : 4635575
-7.54352 -0.852543 -2.35102
-6.01018 -0.00341684 -2.23021
Start OCtree Search.....
==========================================
Ana: 14050, execNum: 11952
Dist; 0.000779929, offset: 1.87431
anaNumfromDreal: 6376.75
Ratio_Ana: 1.17554
==========================================
Ratio for Alpha Control : 2.20332, Diff : -2098
Number of Original Vertices : 4635575
Number of setting Particles : 10213632( 9271150 + 942482 )
Number of Particles: 5145420
** File ../SPBR_DATA/../SPBR_DATA/borobudur/relief_ship//out-relief_ship.xyz.spbr  is generated.

Feature extraction type
Normal point feature extraction: 0, Adaptive point feature extraction: 1
Select an ID >> 1
Feature extraction type ==> 1

Coloring type
Normal: 0, Adaptive: 1
Select an ID >> 0

Highlight color
ORIGINAL: 0, RED: 1, BLACK: 2, CYAN: 3
Select an ID >> 1
Highlight color ==> 1

Creating Octree... (Number of Vertex : 4635575
-7.54352 -0.852543 -2.35102
-6.01018 -0.00341684 -2.23021

Highlighting precision
Input 1/local-area_radius (recommend range [100-600]) >> 300
Input function switching threshold s_th>> 0.16

Input Type(b) function parameters
Maximum opacity α_max in range [0.2, 1] >> 1
Large feature value threshold F_th in range [0.02, 1] >> 1
Point proliferation exponent d >> 2

Write a parameter list ParameterList_Type(b).txt

Input Type(c) function parameters
Maximum opacity α_max in range [0.2, 1] >> 1
Large feature value threshold F_th in range [0.02, 1] >> 0.2
Point proliferation exponent d >> 2

Write a parameter list ParameterList_Type(c).txt

Start OCtree Search.....
10000000	particles (87.6915) %
** File ../SPBR_DATA/../SPBR_DATA/borobudur/relief_ship//out-relief_ship.xyz.spbr_f.spbr  is generated.
Number of Particles (Total): 16549030

Keyboard menu:
  o-key: object control, l-key: light control
  s-key: snapshot image (BMP)
  S-key: snapshot image (PPM)
  G-key: snapshot image (PGM)
  q-key: quit
```