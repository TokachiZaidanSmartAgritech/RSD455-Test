# RSD455-Test
Intel RealSense Depth Camera D455 を使用して馬鈴薯の測定を検証しました。  
Intel RealSense Depth Camera D455とAPIを使用して馬鈴薯を測定する動画にて使用したプログラムを紹介します。

## 開発環境
使用言語 : C++  
Intel RealSense SDK : https://dev.intelrealsense.com/docs/docs-get-started

## プログラム
プログラムは以下の5つです。
1. DepthMeasurement.cpp  : RealSense Depth Cameraから画像の中心までの深度を出力（単位はm）
1. DepthColorView.cpp    : 取得した画像に深度を色付けして表示した画像
1. ColorizerChange.cpp   : DepthColorView.cppで表示した画像の深度の色付けを変更した画像
1. ThresholdFilterChange : 取得した画像に深度閾値フィルターを0.15m～2.5mに設定しそれ以外をマスクした画像
1. PotatoMeasurement.cpp : RealSense Depth Cameraにて取得した物体の横幅、縦幅、奥行き幅を表示（単位はcm）
