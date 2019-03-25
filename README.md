# RcLab-PongBotQ
Simulation for PongBotQ

Step 1. Clone or download를 클릭하여 url을 복사한다.

Step 2. NetBeans의 Team > Remote > clone 을 누른후, Repository URL을 https://github.com/swan0421/RcLab-PongBotQ.git 으로 설정한다.
        (만약, NetBeans에서 Team > Remote > clone 경로가 보이지 않는 경우, NetBeans 화면 좌측에 있는 Projects 패널에서 catkin_ws 를 클릭하면
         보이며, 위의 경로는 git에 연동되었을 때 활성화되는 경로이므로 처음 연동하는 것이라면, Team > git > clone으로 해도 됨)
        User에는 GitHUB의 user_name을 쓰고, Password에는 GitHUB의 비밀번호를 입력한 후 NEXT를 누른다.
        
Step 3. Select Remote Branches를 master* 로 선택하고 Next를 누른다.

Step 4. Parent Directory를 사용자의 home/user_name/catkin_ws/src 경로로 설정하고,
        Clone name을 사용자가 원하는 이름으로 설정하고, (참고 : Clone Name은 패키지에 관련된 이름으로 써서 다른 폴더들과 구별 지을 것)
        Checkout Branch는 master* 로 설정하고,
        Remote Name은 origin으로 설정한 후 Finish를 누른다.
        
Step 5. 사용자의 catkin_ws/src 위치에 Step4에서 설정한 Clone Name 을 갖는 폴더가 있는지 확인하고, 
        폴더 내부에 패키지 구성 파일들(world 폴더, src 폴더, launch 폴더 등)과 model 파일(=PONGBOT_Q_V1)이 있는지 확인한다.

Step 6. 'PONGBOT_Q_V1'폴더를 HOME > .gazebo > models 폴더로 가져와서 시뮬레이션을 위한 파일 셋팅을 마무리한다.
         (.gazebo 폴더가 보이지 않으면, Ctrl+H 를 눌러서 폴더 숨김 해제를 할 것)
         
Step 7. 패키지를 컴파일하기 위해 Netbeans에서 터미널 창을 열고. catkin_make(또는 cm)을 입력하여 컴파일을 진행한다.
        (터미널 창이 없다면, Netbeans의 상단 Winodow > IDE Tools > Termianl 을 클릭)

Step 8. 시뮬레이션 실행을 위해 터미널 창에 roslaunch pongbot_q_v1 pongbot_q_v1.launch 를 입력한다.

이로써, 시뮬레이션이 진행된다.


