# RoadRunner Scenario - Simulink HandsOn
<!-- This is the "Title of the contribution" that was approved during the Community Contribution Review Process --> 

[![View <File Exchange Title> on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/####-file-exchange-title)  
<!-- Add this icon to the README if this repo also appears on File Exchange via the "Connect to GitHub" feature --> 

本リポジトリはRoadRunner ScenarioとMATLAB&reg;/Simulink&reg;を連携させるサンプルを提供しています。
実際の使用方法についての解説は[ウェビナーのアーカイブ後半パート](https://jp.mathworks.com/videos/super-practicalmodel-based-development-and-simulation-including-3d-driving-environment-1723108106305.html)にて公開されていますので、ご視聴いただきながら実行してください。（時間の関係上、全てのコンテンツについて解説できてはおりません。）
また、RoadRunnerとRoadRunner Scenarioの基本的な操作を理解していることが前提になっています。上リンクのウェビナーの前半パートも合わせてご覧ください。


それぞれの主なファイルの説明は下記の通りです。
- handsOn00_liveEditor_demo.mlx: liveEditorの基本操作を体験
- handsOn01_launchRR.mlx: RoadRunnerをMATLABから起動、実行する方法を体験
- handsOn02_changeParameters.mlx: MATLABからRoadRunnerの変数を変更する方法を体験。変数を変更し、forループで繰り返し実行することでシナリオのバリエーション実行が可能。
- handsOn03_simulinkCosimulation.mlx: シンプルなSimulinkモデルとのcosimulationを体験。毎ステップごとに固定値進むだけの簡易的な移動モデルをSimulinkモデルで作成し、RoadRunner Sceanrioとcosimulation.
- handsOn04_sensorSimulaiton.mlx: 固定距離を移動するモデルにセンサモデルを追加。LiDARセンサをはじめに、カメラやミリ波センサを車両に追加し、その計測値の可視化を体験。
- handsOn05_simulaitonPathFollowing.mlx: RoadRunner ScenarioReaderブロックより車速や軌跡情報を取得し、作成した軌跡通りに車両が追従するSimulinkモデルを体験。2輪モデルの車両ダイナミクスをStanley制御でコントロールするモードも実行し、それぞれの軌跡の差を可視化。

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
2．handsOn00*からhandsOn05*まで順に実行<br>


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
MATLAB R2024a、RoadRunner R2024aでテストしています。
MATLABとRoadRunnerのversionは揃える必要があるため、異なるversionをインストールしている方はversionを揃えてください。
- MATLAB
- Simulink
- Automated Driving Toolbox&trade;
- Computer Vision Toolbox&trade; (Automated Driving Toolboxの前提)
- Image Processing Toolbox&trade; (Computer Vision Toolboxの前提)
- RoadRunner
- RoadRunner Scenario
- Vehicle Dynamics Blockset&trade; (handsOn05*実行時のみ必要)

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
