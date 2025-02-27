# RoadRunner Scenario - Simulink HandsOn
[![View <File Exchange Title> on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/####-file-exchange-title)  
<!-- Add this icon to the README if this repo also appears on File Exchange via the "Connect to GitHub" feature --> 

日本語による説明が英語説明の後に続きます。

This repository provides sample use cases for integrating RoadRunner Scenario with MATLAB&reg; & Simulink&reg;.  
For an explanation of how to use these files, please refer to the second half of [this webinar archive (Japanese)](https://jp.mathworks.com/videos/super-practicalmodel-based-development-and-simulation-including-3d-driving-environment-1723108106305.html). It walks you through how to execute them. (Due to time constraints, not all the content is fully explained.)  
Also note that it is assumed you already understand the basic operations of RoadRunner and RoadRunner Scenario. Please watch the first half of the webinar at the above link as well.
 
Below is a description of the main files:
 
- **handsOn00_liveEditor_demo.mlx**: Experience the basic operations of Live Editor.  
- **handsOn01_launchRR.mlx**: Learn how to launch and run RoadRunner from MATLAB.  
- **handsOn02_changeParameters.mlx**: Learn how to change RoadRunner variables from MATLAB. By changing these variables and running a for-loop for repeated execution, you can run scenario variations.  
- **handsOn03_simulinkCosimulation.mlx**: Experience co-simulation with a simple Simulink model. You will create a simple moving model in Simulink, which moves at a fixed distance every step, and co-simulate with RoadRunner Scenario.  
- **handsOn04_sensorSimulaiton.mlx**: Add a sensor model to a model that moves a fixed distance. Start with a LiDAR sensor, then add a camera and millimeter-wave sensor to the vehicle and visualize the measurement data.  
- **handsOn04_2_sensorSimulaitonObserver.mlx**: Obtain and visualize data from each sensor model using Observer (R2024b or later). The vehicle moves as designed in RoadRunner Scenario while sensor data is collected.  
- **handsOn05_simulaitonPathFollowing.mlx**: Use the RoadRunner ScenarioReader block to obtain speed and trajectory information, and then experience a Simulink model in which the vehicle follows the created trajectory. Also try the mode where a bicycle vehicle dynamics model is controlled by Stanley control, and compare the difference in each trajectory.
- **handsOn06_simulaitonPathFollowing14DoF.mlx**: Use the RoadRunner ScenarioReader to obtain speed and trajectory information, and then experience a Simulink model in which the vehicle follows the cretaed trajectory. Also try the mode where a 14 DOF full vehicle model (6 DOF for the vehicle body + 2  DOF × 4 tires) is controlled by Stanley control, and compare the difference in each trajectory. Compare in a scene and scenario that enables you to see effects such as tire saturation and road gradients, which cannot be represetnted by a bicycle model.
- **handsOn06_2_handsOn06_2_simulaitonPathFollowing14DoF_CRG**: similar example to "handsOn06_simulaitonPathFollowing14DoF.mlx" but not 2D Look up table but CRG file is used for ground height reading
- **handsOn07_simulaitonPathFollowingMultibody.mlx**: Use the RoadRunner ScenarioReader to obtain speed and trajectory information, and then experience a Simulink model in which the vehicle follows the cretaed trajectory. This is a sample that uses a Simscape Multibody model for vehicle dynamics.
- **suppelement/CP_estimation_fromTIR/sample_estCP.mlx**: A program that visualizes tire characteristics represented by the Magic Formula from a TIR file, and calculates the cornering power (CP) from the linear region. (Front CP is required as a control parameter for Stanley control.)

## Setup
### Execution Steps
1. Create the RoadRunner project folder in advance.  
2. Execute in order from `handsOn00*` through `handsOn07*`.  
 
### Important Notes
If HTTP_PROXY or HTTPS_PROXY environment variables for a proxy server are set, MATLAB may be unable to launch RoadRunner (i.e., RoadRunner function fails to connect MATLAB and RoadRunner).  
In such cases, please remove the HTTP_PROXY/HTTPS_PROXY environment variable settings, restart, and try again.
 
### Product Configuration (https://www.mathworks.com)
Tested with MATLAB R2024b and RoadRunner R2024b.  
MATLAB and RoadRunner versions must match. If you have installed different versions, please align them.
- MATLAB
- Simulink
- Automated Driving Toolbox&trade;
- Computer Vision Toolbox&trade; (required by Automated Driving Toolbox)
- Image Processing Toolbox&trade; (required by Computer Vision Toolbox)
- RoadRunner
- RoadRunner Scenario
- Vehicle Dynamics Blockset&trade; (only needed when running `handsOn05* and handsOn06*')
- Simulink&reg; 3D Animation&trade; (only needed when enabling 3D engine viewer in handsOn06*)
- Simscape&trade; (only needed when runnning handsOn07*)
- Simscape Multibody&trade; (only needed when runnning handsOn07*)
- Stateflow&reg; (only needed when runnning handsOn07*)

<!-- This is the "Title of the contribution" that was approved during the Community Contribution Review Process --> 


本リポジトリはRoadRunner ScenarioとMATLAB & Simulinkを連携させるサンプルを提供しています。
実際の使用方法についての解説は[ウェビナーのアーカイブ後半パート](https://jp.mathworks.com/videos/super-practicalmodel-based-development-and-simulation-including-3d-driving-environment-1723108106305.html)にて公開されていますので、ご視聴いただきながら実行してください。（時間の関係上、全てのコンテンツについて解説できてはおりません。）
また、RoadRunnerとRoadRunner Scenarioの基本的な操作を理解していることが前提になっています。上リンクのウェビナーの前半パートも合わせてご覧ください。


それぞれの主なファイルの説明は下記の通りです。
- **handsOn00_liveEditor_demo.mlx**: liveEditorの基本操作を体験
- **handsOn01_launchRR.mlx**: RoadRunnerをMATLABから起動、実行する方法を体験
- **handsOn02_changeParameters.mlx**: MATLABからRoadRunnerの変数を変更する方法を体験。変数を変更し、forループで繰り返し実行することでシナリオのバリエーション実行が可能。
- **handsOn03_simulinkCosimulation.mlx**: シンプルなSimulinkモデルとのcosimulationを体験。毎ステップごとに固定値進むだけの簡易的な移動モデルをSimulinkモデルで作成し、RoadRunner Sceanrioとcosimulation.
- **handsOn04_sensorSimulaiton.mlx**: 固定距離を移動するモデルにセンサモデルを追加。LiDARセンサをはじめに、カメラやミリ波センサを車両に追加し、その計測値の可視化を体験。
- **handsOn04_2_sensorSimulaitonObserver.mlx**: 各センサモデルの取得と可視化をObserver(R2024b以降)により実現。車両はRoadRunner Scenarioで設計した通りに動きセンサデータを取得。
- **handsOn05_simulaitonPathFollowing.mlx**: RoadRunner ScenarioReaderブロックより車速や軌跡情報を取得し、作成した軌跡通りに車両が追従するSimulinkモデルを体験。2輪モデルの車両ダイナミクスをStanley制御でコントロールするモードも実行し、それぞれの軌跡の差を可視化。
- **handsOn06_simulaitonPathFollowing14DoF.mlx**: RoadRunner ScenarioReaderブロックより車速や軌跡情報を取得し、作成した軌跡通りに車両が追従するSimulinkモデルを体験。14自由度（車体6自由度＋タイヤ２自由度×4）車両ダイナミクスをStanley制御でコントロールするモードを実行し、それぞれの軌跡の差を可視化。2輪モデルでは表現できないタイヤの飽和領域や道路勾配の影響を可視化できるシーンで比較。
- **handsOn06_2_simulaitonPathFollowing14DoF_CRG.mlx**: handsOn06_simulaitonPathFollowing14DoF.mlxとほとんど同じ例題だが、路面高さの読出しをCRGファイルより実施
- **handsOn07_simulaitonPathFollowingMultibody.mlx**: RoadRunner ScenarioReaderブロックより車速や軌跡情報を取得し、作成した軌跡通りに車両が追従するSimulinkモデルを体験。車両ダイナミクスとしてSimspace Multibodyのモデルを使用するサンプル。
- **suppelement/CP_estimation_fromTIR/sample_estCP.mlx**: TIRファイルよりマジックフォーミュラで表現されたタイヤ特性を可視化し、線形領域よりコーナリングパワー(CP)を算出するプログラム。（stanley制御の制御パラメータとしてフロントCPが必要。）

<!--- If you mention any trademarks, all MathWorks&reg; (including MATLAB&reg;)  and 3rd party trademarks&trade; need to be correctly marked the first time they are prominently used in each file (including the README.MD).  --->
<!--- Markdown supports the following HTML entities: © - &copy;  ® - &reg;  ™ - &trade;
More information about Trademarks can be found internally within the Checklist for Community Contributions and Supportfiles Confluence page--->

<!--- 
If your repository plans to accept contributions, you should include the `CONTRIBUTING.md` file from this repository.  If you **do not accept contributions**, don't copy the `CONTRIBUTING.md` file.

Please remember to delete all template related text that you are not using within your README.md
--->

<!--- Please remember to delete all template related text that you are not using within your README.md ---> 

## Setup
### 実行手順
1．RoadRunnerのプロジェクトフォルダを事前に作成<br>
2．handsOn00*からhandsOn07*まで順に実行<br>

### 注意
プロキシサーバーの環境変数（HTTP_PROXY、HTTPS_PROXY）設定によって、
MATLABからRoadRunnerが起動できない場合があります。（roadrunner関数を使用した際にMATLABとRoadRunner間の通信が接続されない。）
お手数ですが、プロキシサーバーの環境変数（HTTP_PROXY、HTTPS_PROXY）設定を削除し、再起動してからの実行をお願いします。

<!---  
To Run:

1. Install MATLAB and related toolbox
2. Install RoadRunner and activate it. (RoadRunner and RoadRunner Scenario are required.)
3. 
--->

### 製品構成 (https://www.mathworks.com)
MATLAB R2024b、RoadRunner R2024b(Update1以降)でテストしています。
MATLABとRoadRunnerのversionは揃える必要があるため、異なるversionをインストールしている方はversionを揃えてください。
- MATLAB
- Simulink
- Automated Driving Toolbox;
- Computer Vision Toolbox; (Automated Driving Toolboxの前提)
- Image Processing Toolbox; (Computer Vision Toolboxの前提)
- RoadRunner
- RoadRunner Scenario
- Vehicle Dynamics Blockset (handsOn05*, handsOn06*実行時のみ必要)
- Simulink 3D Animation (handsOn06*の詳細表示を有効にした際必要)
- Simscape (handsOn07*実行時のみ必要)
- Simscape Multibody (handsOn07*実行時のみ必要)
- Stateflow (handsOn07*実行時のみ必要)

<!---
### MathWorks Products (https://www.mathworks.com)
Requires MATLAB release R2024a or newer
- [Product1](https://url-to-product1)
- [Product1](https://url-to-product1)
--->

<!--- 3rd Party Products are not included
### 3rd Party Products:
3p:
- [Product1](https://url-to-product1)
- [Product2](https://url-to-product2)
--->

<!--- 
## Installation (Optional)
Installation instuctions

Before proceeding, ensure that the below products are installed:  
* [Product1](https://url-to-product1) 

Please see the [documentation](Documentation/Installation.md) for detailed installation instructions. 

1. Step 1
2. Step 2
--->
<!--- Make sure you have a Installation.md document in the Documentation folder if you are to follow this formatting.  You can choose your own folder formatting if you prefer --->

<!--- This is for Repos that utillize Releases in GitHub --->
<!--- 
## Deployment Steps (Optional) 

To view instructions for deploying <insert repo name>, select a MATLAB release: 
| Release |
| ------- |
| [R2024a](releases/R2024a/README.md) |
| [R2023b](releases/R2023b/README.md) |
| [R2023a\_and\_older](releases/R2023a_and_older/README.md) |
--->

<!--- List or link to any relevent Documentation to help the user Get Started --->

<!--- Examples are not included in this repo
## Examples
To learn how to use this in testing workflows, see [Examples](/examples/). 
--->
<!--- Make sure you have a repo set up correctly if you are to follow this formatting --->


## License
<!--- Make sure you have a License.txt within your Repo --->
The license is available in the [license.txt](license.txt) file in this GitHub repository.

## Community Support
[MATLAB Central](https://www.mathworks.com/matlabcentral)

Copyright 2024 The MathWorks, Inc.

<!--- Do not forget to the add the SECURITY.md to this repo --->
<!--- Add Topics #Topics to your Repo such as #MATLAB  --->

<!--- This is my comment --->

<!-- Include any Trademarks if this is the first time mentioning trademarked products (For Example:  MATLAB&reg; Simulink&reg; Trademark&trade; Simulink Test&#8482;) --> 

