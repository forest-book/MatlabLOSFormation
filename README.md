# MatlabLOSFormation
MATLABで書かれたコードをPythonへと書き換え

## 今回主に必要なライブラリ群
>Numpy<br>
>Scipy<br>
>Matplotlib<br>
>Sympy<br>
>Python-control<br>
>slycot<br>
>coppeliasim-zmqremoteapi-client<br>

## conda コマンドでのモジュールインストール
特に苦戦しがちな**Python-control**,**slycot**,**coppeliasim-zmqremoteapi-client**の<br>
condaコマンドでのインストール方法（他は特に問題なくできる）<br>
```
conda install -c conda-forge coppeliasim-zmqremoteapi-client
conda install -c conda-forge control slycot
```

## 仮想環境の作成
Anaconda Promptにて<br>
```
conda create -p 仮想環境を作成したい場合のフルパス python=バージョン
```

具体例<br>
```
conda create -p E:\塩田\LOS制御におけるチョークポイント通過\MatlabLOSFormation\env python=3.12
```

PowerShellやコマンドプロンプトでPythonを実行する場合<br>
Anaconda Promptにて
```
conda init
```
を実行

**conda init**実行後、PowerShellにて

>. : このシステムではスクリプトの実行が無効になっているため、ファイル `C:\~~~\WindowsPowerShell\profile.ps1` 
>を読み込むことができません。詳細については、
>「about_Execution_Policies」(https://go.microsoft.com/fwlink/?LinkID=135170)
>を参照してください。<br>
>発生場所 行:1 文字:3<br>
>`+` . `'C:\~~~\WindowsPowerShell\profile.ps1'`<br>
>`+`   `~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`<br>
>    &emsp;`+` CategoryInfo          : セキュリティ エラー: (: ) []、PSSecurityException<br>
>    &emsp;`+` FullyQualifiedErrorId : UnauthorizedAccess

<br>が出る場合がある<br>
その時は、PowerShellを管理者として実行し
```
Set-ExecutionPolicy RemoteSigned -Scope CurrentUser
```
を入力（確認メッセージが出たら`Y`を入力）

## 仮想環境の起動コマンド
powershell,コマンドプロンプトで <br>
```
conda activate E:\塩田\LOS制御におけるチョークポイント通過\MatlabLOSFormation\env
```

## 仮想環境の終了コマンド
powershell,コマンドプロンプトで<br>
```
conda deactivate
```

## インストール済みモジュールの確認
```
conda list
```

## Regular API reference(URL)
```
https://manual.coppeliarobotics.com/en/apiFunctions.htm
```
