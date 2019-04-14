# roskeigan_motor_usb

## Installation
<p>rosのworkspaceに移動</p>  

    $ cd catkin_ws/src
    
<p>パッケージをインストール</p>

    $ git clone https://github.com/keigan-motor/roskeigan_motor_usb.git
    $ cd ..
    $ catkin_make

## Demo
<p>launchファイルから実行</p>

    $ roslaunch roskeigan_motor_usb keigan_ros_node_sample.launch

## その他
<p>
制御コマンドと引数の説明は下記ローレベルAPIを参照<br/>

[https://document.keigan-motor.com/software_dev/lowapis/motor_action](https://document.keigan-motor.com/software_dev/lowapis/motor_action)

</p>
<p>
テスト版の為、現時点で実装されている制御コードは、KMControllerROS.py::72行目以下を参照
</p>
