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

    $ roslaunch roskeigan_motor_usb usb_mode.launch
    
<p>ノードグラフ作成(新しいターミナルを開く)</p>

    $ rqt_graph
    
<p>トピック確認</p>

    $ rostopic list
    
<p>トピックの中身を確認</p>

    $ rostopic echo /motor_command
    
    $ rostopic echo /rot_state
    
